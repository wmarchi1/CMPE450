import time
from rplidar import RPLidar

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


PORT = "/dev/ttyUSB0"

FRONT_LIMIT_MM = 2000
FRONT_FOV_DEG = 60
MAX_DISTANCE_MM = 4000


macro_path = [
    (0, 0, 1.0, 0),
    (1, 0, 1.0, 0),
    (2, 0, 1.0, 0),
    (3, 0, 1.0, 0),
    (4, 0, 1.0, 0),
    (5, 0, 1.0, 0),
    (6, 0, 1.0, 0),
    (7, 0, 1.0, 0),
    (8, 0, 1.0, 0),
    (9, 0, 1.0, 0),
    (10, 0, 1.0, 0),
]


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


def find_merge_goal(current, macro_path):
    cx, cy, _, _ = current

    candidates = []

    for point in macro_path:
        x, y, _, _ = point
        if x > cx:
            candidates.append(point)

    if not candidates:
        return macro_path[-1]

    index = min(2, len(candidates) - 1)
    return candidates[index]


def generate_micro_path(current, macro_path, obs):
    x, y, speed, heading = current
    merge_goal = find_merge_goal(current, macro_path)

    if not obs["detected"]:
        return []

    if obs["left"] > obs["right"]:
        return [
            (x, y + 1, speed, heading),
            (x + 1, y + 1, speed, heading),
            (x + 2, y + 0.5, speed, heading),
            merge_goal,
        ]

    return [
        (x, y - 1, speed, heading),
        (x + 1, y - 1, speed, heading),
        (x + 2, y - 0.5, speed, heading),
        merge_goal,
    ]


def save_final_plot(filename="final_path.png"):
    if not history:
        print("No path history to save.")
        return

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        "Autonomous Vehicle Obstacle Avoidance Final Path",
        fontsize=14,
        fontweight="bold"
    )

    # Panel 1: Position Map
    ax_map = fig.add_subplot(2, 2, 1)
    ax_map.set_title("Position Map")
    ax_map.set_xlabel("X Position")
    ax_map.set_ylabel("Y Position")
    ax_map.grid(True, alpha=0.3)
    ax_map.set_aspect("equal")

    macro_x = [p[0] for p in macro_path]
    macro_y = [p[1] for p in macro_path]

    ax_map.plot(macro_x, macro_y, "r--", linewidth=2, label="Macro Path")
    ax_map.plot(macro_x, macro_y, "ro", markersize=7)

    for i, point in enumerate(macro_path):
        ax_map.annotate(
            f"P{i}",
            (point[0], point[1]),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=8
        )

    hist_x = [p[0] for p in history]
    hist_y = [p[1] for p in history]

    ax_map.plot(hist_x, hist_y, color="steelblue", linewidth=2.5, label="Path Taken")
    ax_map.plot(hist_x[-1], hist_y[-1], "bo", markersize=10, label="Final Position")

    for micro_path in micro_paths_used:
        mx = [p[0] for p in micro_path]
        my = [p[1] for p in micro_path]
        ax_map.plot(mx, my, color="limegreen", linewidth=2, alpha=0.8)

    ax_map.legend()

    # Panel 2: Front Distance
    ax_front = fig.add_subplot(2, 2, 2)
    ax_front.set_title("Front Obstacle Distance")
    ax_front.set_xlabel("Scan Step")
    ax_front.set_ylabel("Distance (mm)")
    ax_front.grid(True, alpha=0.3)

    ax_front.plot(front_history, color="orange", linewidth=2, label="Front Distance")
    ax_front.axhline(FRONT_LIMIT_MM, color="red", linestyle="--", label="2m Trigger")
    ax_front.legend()

    # Panel 3: Avoidance Events
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

    ax_events.text(
        0.05,
        0.75,
        summary,
        fontsize=12,
        verticalalignment="top"
    )

    # Panel 4: Macro vs Actual Path
    ax_compare = fig.add_subplot(2, 2, 4)
    ax_compare.set_title("Macro Path vs Actual Path")
    ax_compare.set_xlabel("X Position")
    ax_compare.set_ylabel("Y Position")
    ax_compare.grid(True, alpha=0.3)
    ax_compare.set_aspect("equal")

    ax_compare.plot(macro_x, macro_y, "r--", linewidth=2, label="Macro Path")
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
    lidar = RPLidar(PORT)
    patch_lidar_health(lidar)

    current = macro_path[0]
    macro_index = 0

    active_micro_path = []
    micro_index = 0
    scan_count = 0

    try:
        print("Starting fake macro path simulation...")
        print("Press Ctrl + C to stop and save final PNG.\n")

        try:
            lidar.stop()
            lidar.stop_motor()
            time.sleep(1)
            lidar.clear_input()
        except Exception:
            pass

        for scan in lidar.iter_scans(max_buf_meas=5000):
            scan_count += 1

            obs = read_lidar(scan)

            if active_micro_path:
                current = active_micro_path[micro_index]
                micro_index += 1

                if micro_index >= len(active_micro_path):
                    active_micro_path = []
                    micro_index = 0

            elif obs["detected"]:
                active_micro_path = generate_micro_path(current, macro_path, obs)
                micro_index = 0

                if active_micro_path:
                    micro_paths_used.append(active_micro_path.copy())
                    current = active_micro_path[micro_index]
                    micro_index += 1

            else:
                if macro_index < len(macro_path) - 1:
                    macro_index += 1
                    current = macro_path[macro_index]

            history.append(current)
            front_history.append(obs["front"])

            if scan_count % 10 == 0:
                print(
                    {
                        "step": scan_count,
                        "current": format_coord(current),
                        "obstacle": obs["detected"],
                        "front_mm": round(obs["front"], 1),
                    }
                )

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

        print("LiDAR disconnected.")


if __name__ == "__main__":
    main()
