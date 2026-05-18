import time
import struct
import serial
from rplidar import RPLidar

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


GIGA_PORT = "/dev/ttyACM0"
LIDAR_PORT = "/dev/ttyUSB0"
BAUD = 115200

FRONT_LIMIT_MM = 2000
FRONT_FOV_DEG = 60
MAX_DISTANCE_MM = 4000

COORD_STRUCT = "<ffff"  # x, y, speed, heading


history = []
front_history = []
micro_paths_used = []


def patch_lidar_health(lidar):
    original_get_health = lidar.get_health

    def fixed_get_health():
        result = original_get_health()
        if isinstance(result, tuple) and len(result) > 2:
            return result[0], result[1]
        return result

    lidar.get_health = fixed_get_health


def format_coord(coord):
    x, y, speed, heading = coord
    return f"({x:.2f}, {y:.2f}, {speed:.2f}, {heading:.2f})"


def read_coord_binary(ser):
    data = ser.read(16)

    if len(data) != 16:
        return None

    return struct.unpack(COORD_STRUCT, data)


def write_coord_binary(ser, coord):
    data = struct.pack(COORD_STRUCT, *coord)
    ser.write(data)
    ser.flush()


def wait_for_text(ser, target, timeout=2.0):
    start = time.time()

    while time.time() - start < timeout:
        line = ser.readline().decode(errors="ignore").strip()

        if line == target:
            return True

    return False


def get_giga_position_and_target(ser):
    ser.reset_input_buffer()
    ser.write(b"Start\n")
    ser.flush()

    if not wait_for_text(ser, "ACK_START", timeout=2.0):
        return None, None

    current = read_coord_binary(ser)
    target = read_coord_binary(ser)

    return current, target


def send_micro_point_to_giga(ser, coord):
    ser.reset_input_buffer()
    ser.write(b"Ready\n")
    ser.flush()

    if not wait_for_text(ser, "Initiate", timeout=2.0):
        print("GIGA did not send Initiate")
        return False

    write_coord_binary(ser, coord)

    if wait_for_text(ser, "Done", timeout=2.0):
        return True

    return False


def read_lidar(scan):
    front = []
    left = []
    right = []

    half_fov = FRONT_FOV_DEG / 2

    for _, angle, distance in scan:
        if distance <= 0 or distance > MAX_DISTANCE_MM:
            continue

        if angle >= 360 - half_fov or angle <= half_fov:
            front.append(distance)
        elif half_fov < angle < 150:
            left.append(distance)
        elif 210 < angle < 360 - half_fov:
            right.append(distance)

    front_min = min(front) if front else 99999
    left_avg = sum(left) / len(left) if left else 99999
    right_avg = sum(right) / len(right) if right else 99999

    return {
        "detected": front_min <= FRONT_LIMIT_MM,
        "front": front_min,
        "left": left_avg,
        "right": right_avg,
    }


def generate_micro_path(current, target, obs):
    x, y, speed, heading = current
    target_x, target_y, target_speed, target_heading = target

    merge_goal = target

    if not obs["detected"]:
        return []

    if obs["left"] > obs["right"]:
        return [
            (x, y + 1.0, speed, heading),
            (x + 1.0, y + 1.0, speed, heading),
            (x + 2.0, y + 0.5, speed, heading),
            merge_goal,
        ]

    return [
        (x, y - 1.0, speed, heading),
        (x + 1.0, y - 1.0, speed, heading),
        (x + 2.0, y - 0.5, speed, heading),
        merge_goal,
    ]


def save_final_plot(filename="final_path.png"):
    if not history:
        print("No path history to save.")
        return

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("GIGA + RPLIDAR Obstacle Avoidance Final Path", fontsize=14, fontweight="bold")

    ax_map = fig.add_subplot(2, 2, 1)
    ax_map.set_title("Position Map")
    ax_map.set_xlabel("X Position")
    ax_map.set_ylabel("Y Position")
    ax_map.grid(True, alpha=0.3)
    ax_map.set_aspect("equal")

    hist_x = [p[0] for p in history]
    hist_y = [p[1] for p in history]

    ax_map.plot(hist_x, hist_y, color="steelblue", linewidth=2.5, label="Path Taken")
    ax_map.plot(hist_x[-1], hist_y[-1], "bo", markersize=10, label="Final Position")

    for micro_path in micro_paths_used:
        mx = [p[0] for p in micro_path]
        my = [p[1] for p in micro_path]
        ax_map.plot(mx, my, color="limegreen", linewidth=2, alpha=0.8, label="Micro Path")

    ax_map.legend()

    ax_front = fig.add_subplot(2, 2, 2)
    ax_front.set_title("Front Obstacle Distance")
    ax_front.set_xlabel("Scan Step")
    ax_front.set_ylabel("Distance (mm)")
    ax_front.grid(True, alpha=0.3)
    ax_front.plot(front_history, color="orange", linewidth=2, label="Front Distance")
    ax_front.axhline(FRONT_LIMIT_MM, color="red", linestyle="--", label="2m Trigger")
    ax_front.legend()

    ax_events = fig.add_subplot(2, 2, 3)
    ax_events.set_title("Avoidance Summary")
    ax_events.axis("off")

    summary = (
        f"Total steps: {len(history)}\n"
        f"Micro paths triggered: {len(micro_paths_used)}\n"
        f"Final position: {format_coord(history[-1])}\n"
        f"Front trigger distance: {FRONT_LIMIT_MM} mm\n"
        f"Front FOV: {FRONT_FOV_DEG} degrees"
    )

    ax_events.text(0.05, 0.75, summary, fontsize=12, verticalalignment="top")

    ax_compare = fig.add_subplot(2, 2, 4)
    ax_compare.set_title("Actual Path + Micro Paths")
    ax_compare.set_xlabel("X Position")
    ax_compare.set_ylabel("Y Position")
    ax_compare.grid(True, alpha=0.3)
    ax_compare.set_aspect("equal")

    ax_compare.plot(hist_x, hist_y, color="steelblue", linewidth=2.5, label="Actual Path")

    for micro_path in micro_paths_used:
        mx = [p[0] for p in micro_path]
        my = [p[1] for p in micro_path]
        ax_compare.plot(mx, my, color="limegreen", linewidth=2, alpha=0.8)

    ax_compare.legend()

    plt.tight_layout()
    plt.savefig(filename, dpi=300)
    plt.close(fig)

    print(f"Saved final plot to {filename}")


def main():
    giga = serial.Serial(GIGA_PORT, BAUD, timeout=1)
    lidar = RPLidar(LIDAR_PORT)
    patch_lidar_health(lidar)

    try:
        print("Starting GIGA + RPLIDAR obstacle avoidance.")
        print("Press Ctrl + C to stop and save final PNG.\n")

        time.sleep(2)
        giga.reset_input_buffer()

        try:
            lidar.stop()
            lidar.stop_motor()
            time.sleep(1)
            lidar.clear_input()
        except Exception:
            pass

        for scan in lidar.iter_scans(max_buf_meas=5000):
            current, target = get_giga_position_and_target(giga)

            if current is None or target is None:
                print("Could not read GIGA position/target.")
                continue

            obs = read_lidar(scan)
            history.append(current)
            front_history.append(obs["front"])

            print({
                "current": format_coord(current),
                "target": format_coord(target),
                "obstacle": obs["detected"],
                "front_mm": round(obs["front"], 1),
            })

            if obs["detected"]:
                micro_path = generate_micro_path(current, target, obs)
                micro_paths_used.append(micro_path.copy())

                next_micro_point = micro_path[0]

                print("Obstacle detected. Sending micro point:")
                print(format_coord(next_micro_point))

                success = send_micro_point_to_giga(giga, next_micro_point)

                if success:
                    print("Micro point sent to GIGA.")
                else:
                    print("Failed to send micro point.")

    except KeyboardInterrupt:
        print("\nStopped by user. Saving final plot...")

    finally:
        save_final_plot("final_path.png")

        try:
            lidar.stop()
        except Exception:
            pass

        try:
            lidar.stop_motor()
        except Exception:
            pass

        try:
            lidar.disconnect()
        except Exception:
            pass

        try:
            giga.close()
        except Exception:
            pass

        print("Disconnected.")


if __name__ == "__main__":
    main()
