"""
combinedLidarUltrasonicTest.ino

NOTE:
This file is Python code, but it is named .ino to match the earlier project file style.
Run it on the Raspberry Pi with:

    python3 combinedLidarUltrasonicTest.ino

Purpose:
- Read the RPLidar and 3 ultrasonic sensors at the same time.
- Use sensor fusion to decide if an obstacle is in front.
- Use left/right clearance from BOTH LiDAR and ultrasonic sensors to choose an avoidance side.
- Simulate the macro path, generated micro paths, and actual path taken.
- Save a final graph showing LiDAR distance, ultrasonic distances, and the path.

Required packages on Raspberry Pi:
    sudo apt update
    sudo apt install python3-gpiozero python3-lgpio python3-matplotlib
    python3 -m pip install rplidar-roboticia

Ultrasonic pins used:
    Left:  TRIG=23, ECHO=24
    Front: TRIG=5,  ECHO=6
    Right: TRIG=17, ECHO=27

LiDAR port used:
    /dev/ttyUSB0
"""

import math
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gpiozero import DistanceSensor
from rplidar import RPLidar


# ---------------- HARDWARE SETTINGS ----------------
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 115200

LEFT_TRIGGER = 23
LEFT_ECHO = 24

FRONT_TRIGGER = 5
FRONT_ECHO = 6

RIGHT_TRIGGER = 17
RIGHT_ECHO = 27

ULTRASONIC_MAX_DISTANCE_M = 4.0


# ---------------- AVOIDANCE SETTINGS ----------------
# LiDAR and ultrasonic use different useful ranges, so keep their thresholds separate.
LIDAR_TRIGGER_DISTANCE_MM = 2000
ULTRASONIC_TRIGGER_DISTANCE_MM = 2000

# LiDAR field of view sections.
FRONT_FOV_DEG = 60      # front = -30 to +30 degrees
SIDE_FOV_DEG = 90       # side checks use about 30 to 90 degrees on each side

READ_DELAY_SECONDS = 0.05


# ---------------- TEST MACRO PATH ----------------
# Coordinate format: (x position, y position, speed, heading)
macro_path = [(x, 0, 1.0, 0) for x in range(0, 101)]



# ---------------- HISTORY FOR GRAPHING ----------------
history = []
micro_paths_used = []

lidar_front_history = []
lidar_left_history = []
lidar_right_history = []

ultra_left_history = []
ultra_front_history = []
ultra_right_history = []

front_detect_history = []
avoid_side_history = []


# ---------------- HELPER FUNCTIONS ----------------
def format_coord(coord):
    x, y, speed, heading = coord
    return f"({x:.2f}, {y:.2f}, {speed:.2f}, {heading:.2f})"


def normalize_angle(angle):
    """Convert LiDAR angle to -180 to +180 degrees."""
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle


def safe_min(values, default=99999):
    clean = [v for v in values if v is not None and v > 0]
    if not clean:
        return default
    return min(clean)


def read_ultrasonic_mm(sensor):
    """
    gpiozero DistanceSensor.distance returns a 0.0 to 1.0 scale of max_distance.
    Convert that value into millimeters.
    """
    try:
        return sensor.distance * ULTRASONIC_MAX_DISTANCE_M
    except Exception:
        return 99999


def read_all_ultrasonics(left_sensor, front_sensor, right_sensor):
    left_mm = read_ultrasonic_mm(left_sensor)
    time.sleep(0.01)

    front_mm = read_ultrasonic_mm(front_sensor)
    time.sleep(0.01)

    right_mm = read_ultrasonic_mm(right_sensor)
    time.sleep(0.01)

    return left_mm, front_mm, right_mm


def analyze_lidar_scan(scan):
    """
    Pull useful LiDAR distances from one scan.

    Assumption from the existing LiDAR program:
        0 degrees = forward
        negative angles = left
        positive angles = right

    Returns:
        lidar_left_mm, lidar_front_mm, lidar_right_mm
    """
    front_values = []
    left_values = []
    right_values = []

    half_front = FRONT_FOV_DEG / 2

    for quality, angle, distance in scan:
        if quality == 0 or distance <= 0:
            continue

        heading = normalize_angle(angle)

        if -half_front <= heading <= half_front:
            front_values.append(distance)
        elif -SIDE_FOV_DEG <= heading < -half_front:
            left_values.append(distance)
        elif half_front < heading <= SIDE_FOV_DEG:
            right_values.append(distance)

    lidar_front_mm = safe_min(front_values)
    lidar_left_mm = safe_min(left_values)
    lidar_right_mm = safe_min(right_values)

    return lidar_left_mm, lidar_front_mm, lidar_right_mm


def decide_obstacle_and_side(lidar_left, lidar_front, lidar_right,
                             ultra_left, ultra_front, ultra_right):
    """
    Sensor fusion decision:
    - A front obstacle is detected if LiDAR OR the front ultrasonic sees one.
    - Left and right ultrasonic sensors do not trigger avoidance alone.
      They help decide which side has more room.
    - The side with the larger combined clearance score is selected.
    """
    lidar_front_detected = lidar_front <= LIDAR_TRIGGER_DISTANCE_MM
    ultra_front_detected = ultra_front <= ULTRASONIC_TRIGGER_DISTANCE_MM
    front_detected = lidar_front_detected or ultra_front_detected

    # Clip values so one huge open reading does not overpower everything.
    # left_score = min(lidar_left, 3000) + min(ultra_left, 2000)
    left_score =  min(ultra_left, 2000)
    # right_score = min(lidar_right, 3000) + min(ultra_right, 2000)
    right_score =  min(ultra_right, 2000)

    if left_score >= right_score:
        avoid_side = "left"
    else:
        avoid_side = "right"

    return front_detected, avoid_side, left_score, right_score


def find_merge_goal(current, macro_path_points):
    cx, _, _, _ = current
    candidates = []

    for point in macro_path_points:
        x, _, _, _ = point
        if x > cx:
            candidates.append(point)

    if not candidates:
        return macro_path_points[-1]

    index = min(2, len(candidates) - 1)
    return candidates[index]


def generate_micro_path(current, macro_path_points, avoid_side):
    """
    Generate a simple micro path around the obstacle.

    For this graph:
        left  = positive Y
        right = negative Y
    """
    x, y, speed, heading = current
    merge_goal = find_merge_goal(current, macro_path_points)

    if avoid_side == "left":
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


def save_final_plot(filename="combined_lidar_ultrasonic_final_path.png"):
    if not history:
        print("No history found. Nothing to graph.")
        return

    fig = plt.figure(figsize=(18, 16))
    fig.suptitle(
        "Autonomous Vehicle LiDAR + Ultrasonic Obstacle Avoidance Final Path",
        fontsize=14,
        fontweight="bold",
    )

    macro_x = [p[0] for p in macro_path]
    macro_y = [p[1] for p in macro_path]
    hist_x = [p[0] for p in history]
    hist_y = [p[1] for p in history]

    # 1. Position map
    ax_map = fig.add_subplot(4, 2, 1)
    ax_map.set_title("Position Map")
    ax_map.set_xlabel("X Position")
    ax_map.set_ylabel("Y Position")
    ax_map.grid(True, alpha=0.3)
    ax_map.set_aspect("equal")

    ax_map.plot(macro_x, macro_y, "r--", linewidth=2, label="Macro Path")
    ax_map.plot(macro_x, macro_y, "ro", markersize=6)
    ax_map.plot(hist_x, hist_y, color="steelblue", linewidth=2.5, label="Path Taken")
    ax_map.plot(hist_x[-1], hist_y[-1], "bo", markersize=10, label="Final Position")

    for i, point in enumerate(macro_path):
        ax_map.annotate(
            f"P{i}",
            (point[0], point[1]),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=8,
        )

    for micro_path in micro_paths_used:
        mx = [p[0] for p in micro_path]
        my = [p[1] for p in micro_path]
        ax_map.plot(mx, my, color="limegreen", linewidth=2, alpha=0.8, label="Micro Path")

    handles, labels = ax_map.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax_map.legend(unique.values(), unique.keys())

    # 2. LiDAR front distance
    ax_lidar = fig.add_subplot(4, 2, 2)
    ax_lidar.set_title("LiDAR Front Distance")
    ax_lidar.set_xlabel("Scan Step")
    ax_lidar.set_ylabel("Distance (mm)")
    ax_lidar.grid(True, alpha=0.3)
    ax_lidar.plot(lidar_front_history, color="orange", linewidth=2, label="LiDAR Front")
    ax_lidar.axhline(LIDAR_TRIGGER_DISTANCE_MM, color="red", linestyle="--", label="LiDAR Trigger")
    ax_lidar.legend()

    # 3. Ultrasonic left
    ax_ul = fig.add_subplot(4, 2, 3)
    ax_ul.set_title("Left Ultrasonic Distance")
    ax_ul.set_xlabel("Scan Step")
    ax_ul.set_ylabel("Distance (mm)")
    ax_ul.grid(True, alpha=0.3)
    ax_ul.plot(ultra_left_history, color="purple", linewidth=2, label="Left Ultrasonic")
    ax_ul.axhline(ULTRASONIC_TRIGGER_DISTANCE_MM, color="red", linestyle="--", label="Ultrasonic Trigger")
    ax_ul.legend()

    # 4. Ultrasonic front
    ax_uf = fig.add_subplot(4, 2, 4)
    ax_uf.set_title("Front Ultrasonic Distance")
    ax_uf.set_xlabel("Scan Step")
    ax_uf.set_ylabel("Distance (mm)")
    ax_uf.grid(True, alpha=0.3)
    ax_uf.plot(ultra_front_history, color="darkorange", linewidth=2, label="Front Ultrasonic")
    ax_uf.axhline(ULTRASONIC_TRIGGER_DISTANCE_MM, color="red", linestyle="--", label="Ultrasonic Trigger")
    ax_uf.legend()

    # 5. Ultrasonic right
    ax_ur = fig.add_subplot(4, 2, 5)
    ax_ur.set_title("Right Ultrasonic Distance")
    ax_ur.set_xlabel("Scan Step")
    ax_ur.set_ylabel("Distance (mm)")
    ax_ur.grid(True, alpha=0.3)
    ax_ur.plot(ultra_right_history, color="green", linewidth=2, label="Right Ultrasonic")
    ax_ur.axhline(ULTRASONIC_TRIGGER_DISTANCE_MM, color="red", linestyle="--", label="Ultrasonic Trigger")
    ax_ur.legend()

    # 6. Macro vs actual path
    ax_compare = fig.add_subplot(4, 2, 6)
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
        ax_compare.plot(mx, my, color="limegreen", linewidth=2, alpha=0.8, label="Micro Path")

    handles, labels = ax_compare.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax_compare.legend(unique.values(), unique.keys())

    # 7. LiDAR side distances
    ax_lidar_sides = fig.add_subplot(4, 2, 7)
    ax_lidar_sides.set_title("LiDAR Left/Right Clearance")
    ax_lidar_sides.set_xlabel("Scan Step")
    ax_lidar_sides.set_ylabel("Distance (mm)")
    ax_lidar_sides.grid(True, alpha=0.3)
    ax_lidar_sides.plot(lidar_left_history, linewidth=2, label="LiDAR Left")
    ax_lidar_sides.plot(lidar_right_history, linewidth=2, label="LiDAR Right")
    ax_lidar_sides.legend()

    # 8. Summary
    ax_summary = fig.add_subplot(4, 2, 8)
    ax_summary.set_title("Avoidance Summary")
    ax_summary.axis("off")

    left_count = avoid_side_history.count("left")
    right_count = avoid_side_history.count("right")
    front_count = sum(1 for value in front_detect_history if value)

    summary = (
        f"Total steps: {len(history)}\n"
        f"Front obstacle detections: {front_count}\n"
        f"Micro paths triggered: {len(micro_paths_used)}\n"
        f"Avoided left: {left_count}\n"
        f"Avoided right: {right_count}\n"
        f"Final position: {format_coord(history[-1])}\n"
        f"LiDAR trigger distance: {LIDAR_TRIGGER_DISTANCE_MM} mm\n"
        f"Ultrasonic trigger distance: {ULTRASONIC_TRIGGER_DISTANCE_MM} mm\n"
        f"LiDAR port: {LIDAR_PORT}\n"
        f"Left US pins: TRIG={LEFT_TRIGGER}, ECHO={LEFT_ECHO}\n"
        f"Front US pins: TRIG={FRONT_TRIGGER}, ECHO={FRONT_ECHO}\n"
        f"Right US pins: TRIG={RIGHT_TRIGGER}, ECHO={RIGHT_ECHO}"
    )
    ax_summary.text(0.03, 0.95, summary, fontsize=11, verticalalignment="top")

    plt.tight_layout()
    plt.savefig(filename, dpi=300)
    plt.close(fig)
    print(f"Saved final plot to {filename}")


def main():
    left_sensor = None
    front_sensor = None
    right_sensor = None
    lidar = None

    current = macro_path[0]
    macro_index = 0
    active_micro_path = []
    micro_index = 0
    scan_count = 0

    try:
        print("Starting LiDAR + ultrasonic obstacle avoidance test...")
        print("Press Ctrl + C to stop and save the final graph.\n")

        left_sensor = DistanceSensor(
            echo=LEFT_ECHO,
            trigger=LEFT_TRIGGER,
            max_distance=ULTRASONIC_MAX_DISTANCE_M,
        )
        front_sensor = DistanceSensor(
            echo=FRONT_ECHO,
            trigger=FRONT_TRIGGER,
            max_distance=ULTRASONIC_MAX_DISTANCE_M,
        )
        right_sensor = DistanceSensor(
            echo=RIGHT_ECHO,
            trigger=RIGHT_TRIGGER,
            max_distance=ULTRASONIC_MAX_DISTANCE_M,
        )

        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)

        for scan in lidar.iter_scans():
            scan_count += 1

            lidar_left, lidar_front, lidar_right = analyze_lidar_scan(scan)
            ultra_left, ultra_front, ultra_right = read_all_ultrasonics(
                left_sensor,
                front_sensor,
                right_sensor,
            )

            front_detected, avoid_side, left_score, right_score = decide_obstacle_and_side(
                lidar_left,
                lidar_front,
                lidar_right,
                ultra_left,
                ultra_front,
                ultra_right,
            )

            if active_micro_path:
                current = active_micro_path[micro_index]
                micro_index += 1

                if micro_index >= len(active_micro_path):
                    active_micro_path = []
                    micro_index = 0

            elif front_detected:
                active_micro_path = generate_micro_path(current, macro_path, avoid_side)
                micro_paths_used.append(active_micro_path.copy())
                avoid_side_history.append(avoid_side)
                current = active_micro_path[0]
                micro_index = 1

            else:
                if macro_index < len(macro_path) - 1:
                    macro_index += 1
                    current = macro_path[macro_index]

            history.append(current)

            lidar_left_history.append(lidar_left)
            lidar_front_history.append(lidar_front)
            lidar_right_history.append(lidar_right)

            ultra_left_history.append(ultra_left)
            ultra_front_history.append(ultra_front)
            ultra_right_history.append(ultra_right)

            front_detect_history.append(front_detected)

            if scan_count % 10 == 0:
                print(
                    {
                        "step": scan_count,
                        "position": format_coord(current),
                        "front_detected": front_detected,
                        "avoid_side": avoid_side if front_detected else "none",
                        "lidar_front_mm": round(lidar_front, 1),
                        "ultra_front_mm": round(ultra_front, 1),
                        "left_score": round(left_score, 1),
                        "right_score": round(right_score, 1),
                    }
                )

            time.sleep(READ_DELAY_SECONDS)

    except KeyboardInterrupt:
        print("\nStopped by user. Saving final plot...")

    except Exception as error:
        print(f"\nError: {error}")
        print("Saving graph with whatever data was collected...")

    finally:
        try:
            if lidar is not None:
                lidar.stop()
                lidar.disconnect()
        except Exception:
            pass

        for sensor in [left_sensor, front_sensor, right_sensor]:
            try:
                if sensor is not None:
                    sensor.close()
            except Exception:
                pass

        save_final_plot("combined_lidar_ultrasonic_final_path.png")
        print("Done.")


if __name__ == "__main__":
    main()
