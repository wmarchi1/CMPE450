"""
ultrasonicTest.ino

NOTE:
This file is written in Python, but it is named .ino to match the LiDAR test
file naming style used in this project. Run it with Python on the Raspberry Pi:

    python3 ultrasonicTest.ino

Purpose:
- Test 3 ultrasonic sensors: left, front, and right.
- Simulate a macro path like the LiDAR test program.
- Generate a micro path when an obstacle is detected.
- Save a final graph showing:
    1. Position map
    2. Left ultrasonic distance
    3. Front ultrasonic distance
    4. Right ultrasonic distance
    5. Avoidance summary
    6. Macro path vs actual path
"""

import time
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from gpiozero import DistanceSensor


# ---------------- SENSOR SETTINGS ----------------
# These pins match the original usonictest.py file.
LEFT_ECHO = 24
LEFT_TRIGGER = 23

FRONT_ECHO = 6
FRONT_TRIGGER = 5

RIGHT_ECHO = 27
RIGHT_TRIGGER = 17

MAX_DISTANCE_M = 4.0
TRIGGER_DISTANCE_MM = 2000
READ_DELAY_SECONDS = 0.10


# ---------------- FAKE MACRO PATH ----------------
# Each coordinate is in the format:
# (x position, y position, speed, heading)
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


# ---------------- HISTORY LISTS FOR GRAPHING ----------------
history = []
left_history = []
front_history = []
right_history = []
micro_paths_used = []


# ---------------- HELPER FUNCTIONS ----------------
def format_coord(coord):
    x, y, speed, heading = coord
    return f"({x:.2f}, {y:.2f}, {speed:.2f}, {heading:.2f})"


def distance_to_mm(sensor):
    """Read a gpiozero DistanceSensor value and convert meters to millimeters."""
    try:
        return sensor.distance * MAX_DISTANCE_M * 1000
    except Exception:
        return 99999


def read_ultrasonics(left_sensor, front_sensor, right_sensor):
    """Read the left, front, and right ultrasonic sensors."""
    left_mm = distance_to_mm(left_sensor)
    time.sleep(0.02)

    front_mm = distance_to_mm(front_sensor)
    time.sleep(0.02)

    right_mm = distance_to_mm(right_sensor)
    time.sleep(0.02)

    detected_left = left_mm <= TRIGGER_DISTANCE_MM
    detected_front = front_mm <= TRIGGER_DISTANCE_MM
    detected_right = right_mm <= TRIGGER_DISTANCE_MM

    return {
        "detected": detected_left or detected_front or detected_right,
        "left_detected": detected_left,
        "front_detected": detected_front,
        "right_detected": detected_right,
        "left": left_mm,
        "front": front_mm,
        "right": right_mm,
    }


def find_merge_goal(current, macro_path_points):
    """Find a future macro point to merge back into after a micro path."""
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


def generate_micro_path(current, macro_path_points, obs):
    """
    Generate a small avoidance path.

    Basic idea:
    - If the front sensor sees an obstacle, pick the more open side.
    - If only the left sensor sees an obstacle, move right/down.
    - If only the right sensor sees an obstacle, move left/up.
    """
    x, y, speed, heading = current
    merge_goal = find_merge_goal(current, macro_path_points)

    if not obs["detected"]:
        return []

    # Front obstacle: choose the side with more distance.
    if obs["front_detected"]:
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

    # Left obstacle only: move away to the right/down.
    if obs["left_detected"]:
        return [
            (x, y - 1, speed, heading),
            (x + 1, y - 1, speed, heading),
            merge_goal,
        ]

    # Right obstacle only: move away to the left/up.
    if obs["right_detected"]:
        return [
            (x, y + 1, speed, heading),
            (x + 1, y + 1, speed, heading),
            merge_goal,
        ]

    return []


def save_final_plot(filename="ultrasonic_final_path.png"):
    """Save a final graph similar to the LiDAR test graph."""
    if not history:
        print("No path history to save.")
        return

    fig = plt.figure(figsize=(18, 12))
    fig.suptitle(
        "Autonomous Vehicle Ultrasonic Obstacle Avoidance Final Path",
        fontsize=14,
        fontweight="bold",
    )

    # Panel 1: Position Map
    ax_map = fig.add_subplot(3, 2, 1)
    ax_map.set_title("Position Map")
    ax_map.set_xlabel("X Position")
    ax_map.set_ylabel("Y Position")
    ax_map.grid(True, alpha=0.3)
    ax_map.set_aspect("equal")

    macro_x = [p[0] for p in macro_path]
    macro_y = [p[1] for p in macro_path]
    hist_x = [p[0] for p in history]
    hist_y = [p[1] for p in history]

    ax_map.plot(macro_x, macro_y, "r--", linewidth=2, label="Macro Path")
    ax_map.plot(macro_x, macro_y, "ro", markersize=7)

    for i, point in enumerate(macro_path):
        ax_map.annotate(
            f"P{i}",
            (point[0], point[1]),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=8,
        )

    ax_map.plot(hist_x, hist_y, color="steelblue", linewidth=2.5, label="Path Taken")
    ax_map.plot(hist_x[-1], hist_y[-1], "bo", markersize=10, label="Final Position")

    for micro_path in micro_paths_used:
        mx = [p[0] for p in micro_path]
        my = [p[1] for p in micro_path]
        ax_map.plot(mx, my, color="limegreen", linewidth=2, alpha=0.8, label="Micro Path")

    handles, labels = ax_map.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax_map.legend(unique.values(), unique.keys())

    # Panel 2: Left Distance
    ax_left = fig.add_subplot(3, 2, 2)
    ax_left.set_title("Left Ultrasonic Distance")
    ax_left.set_xlabel("Scan Step")
    ax_left.set_ylabel("Distance (mm)")
    ax_left.grid(True, alpha=0.3)
    ax_left.plot(left_history, color="purple", linewidth=2, label="Left Distance")
    ax_left.axhline(TRIGGER_DISTANCE_MM, color="red", linestyle="--", label="2m Trigger")
    ax_left.legend()

    # Panel 3: Front Distance
    ax_front = fig.add_subplot(3, 2, 3)
    ax_front.set_title("Front Ultrasonic Distance")
    ax_front.set_xlabel("Scan Step")
    ax_front.set_ylabel("Distance (mm)")
    ax_front.grid(True, alpha=0.3)
    ax_front.plot(front_history, color="orange", linewidth=2, label="Front Distance")
    ax_front.axhline(TRIGGER_DISTANCE_MM, color="red", linestyle="--", label="2m Trigger")
    ax_front.legend()

    # Panel 4: Right Distance
    ax_right = fig.add_subplot(3, 2, 4)
    ax_right.set_title("Right Ultrasonic Distance")
    ax_right.set_xlabel("Scan Step")
    ax_right.set_ylabel("Distance (mm)")
    ax_right.grid(True, alpha=0.3)
    ax_right.plot(right_history, color="green", linewidth=2, label="Right Distance")
    ax_right.axhline(TRIGGER_DISTANCE_MM, color="red", linestyle="--", label="2m Trigger")
    ax_right.legend()

    # Panel 5: Avoidance Summary
    ax_summary = fig.add_subplot(3, 2, 5)
    ax_summary.set_title("Avoidance Summary")
    ax_summary.axis("off")

    summary = (
        f"Total steps: {len(history)}\n"
        f"Micro paths triggered: {len(micro_paths_used)}\n"
        f"Final position: {format_coord(history[-1])}\n"
        f"Trigger distance: {TRIGGER_DISTANCE_MM} mm\n"
        f"Left sensor pins: TRIG={LEFT_TRIGGER}, ECHO={LEFT_ECHO}\n"
        f"Front sensor pins: TRIG={FRONT_TRIGGER}, ECHO={FRONT_ECHO}\n"
        f"Right sensor pins: TRIG={RIGHT_TRIGGER}, ECHO={RIGHT_ECHO}"
    )

    ax_summary.text(0.05, 0.90, summary, fontsize=12, verticalalignment="top")

    # Panel 6: Macro Path vs Actual Path
    ax_compare = fig.add_subplot(3, 2, 6)
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

    plt.tight_layout()
    plt.savefig(filename, dpi=300)
    plt.close(fig)

    print(f"Saved final plot to {filename}")


def main():
    left_sensor = DistanceSensor(
        echo=LEFT_ECHO,
        trigger=LEFT_TRIGGER,
        max_distance=MAX_DISTANCE_M,
    )
    front_sensor = DistanceSensor(
        echo=FRONT_ECHO,
        trigger=FRONT_TRIGGER,
        max_distance=MAX_DISTANCE_M,
    )
    right_sensor = DistanceSensor(
        echo=RIGHT_ECHO,
        trigger=RIGHT_TRIGGER,
        max_distance=MAX_DISTANCE_M,
    )

    current = macro_path[0]
    macro_index = 0

    active_micro_path = []
    micro_index = 0
    scan_count = 0

    try:
        print("Starting ultrasonic macro path simulation...")
        print("Press Ctrl + C to stop and save final PNG.\n")

        while True:
            scan_count += 1
            obs = read_ultrasonics(left_sensor, front_sensor, right_sensor)

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
            left_history.append(obs["left"])
            front_history.append(obs["front"])
            right_history.append(obs["right"])

            if scan_count % 10 == 0:
                print(
                    {
                        "step": scan_count,
                        "current": format_coord(current),
                        "obstacle": obs["detected"],
                        "left_mm": round(obs["left"], 1),
                        "front_mm": round(obs["front"], 1),
                        "right_mm": round(obs["right"], 1),
                    }
                )

            time.sleep(READ_DELAY_SECONDS)

    except KeyboardInterrupt:
        print("\nStopped by user. Saving final plot...")

    finally:
        save_final_plot("ultrasonic_final_path.png")

        try:
            left_sensor.close()
            front_sensor.close()
            right_sensor.close()
        except Exception:
            pass

        print("Ultrasonic sensors closed.")


if __name__ == "__main__":
    main()
