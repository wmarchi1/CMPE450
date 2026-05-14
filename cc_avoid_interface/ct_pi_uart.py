import serial
import time

class ArduinoPathClient:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=0.6, startup_delay=2):
        self.TIME_OUT = timeout
        self.ser = serial.Serial(port, baudrate, timeout=self.TIME_OUT)
        time.sleep(startup_delay)
        self.ser.reset_input_buffer()

   def parse_coord(self, line):
        # Parse "(x, y, speed, heading)"
        try:
            values = line[1:-1].split(",")

            x = float(values[0])
            y = float(values[1])
            speed = float(values[2])
            heading = float(values[3])

            return (x, y, speed, heading)

        except Exception:
            return None


    def get_macro_path(self):
        self.ser.write(b"Start\n")

        start = time.time()
        path = []

        while True:
            if time.time() - start > self.TIME_OUT:
                break

            line = self.ser.readline().decode("utf-8").strip()

            if line == "END":
                break

            coord = self.parse_coord(line)

            if coord is not None:
                path.append(coord)

        return path


    def send_micro_path(self, micro_path):
        # Send generated micro path to GIGA
        self.ser.write(b"Ready\n")

        start = time.time()

        while True:
            if time.time() - start > self.TIME_OUT:
                return False

            line = self.ser.readline().decode("utf-8").strip()

            if line == "Initiate":
                data = str(len(micro_path)) + "\n"
                data += "\n".join(micro_path) + "\n"

                self.ser.write(data.encode("utf-8"))

            elif line == "Done":
                return True

            elif line == "Incomplete":
                return False


    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


def read_pi_sensors():
    # detected: 1 means obstacle detected, 0 means path clear
    return {
        "detected": 1,
        "front": 350.0,
        "left": 900.0,
        "right": 500.0
    }


def format_coord(coord):
    # Convert coordinate tuple back into string format for GIGA
    x, y, speed, heading = coord
    return f"({x:.2f}, {y:.2f}, {speed:.2f}, {heading:.2f})"


def find_merge_goal(current, macro_path):
    # Pick the closest point on the macro path to merge back into
    if not macro_path:
        return current

    cx, cy, _, _ = current

    best_point = macro_path[0]
    best_dist = float("inf")

    for point in macro_path:
        x, y, _, _ = point

        dist = abs(x - cx) + abs(y - cy)

        if dist < best_dist:
            best_dist = dist
            best_point = point

    return best_point


def generate_micro_path(current, macro_path, obs):
    # Generate detour waypoints using Pi sensor data
    x, y, speed, heading = current

    merge_goal = find_merge_goal(current, macro_path)

    if obs is None:
        return [format_coord(merge_goal)]

    if obs["detected"] == 1:
        # Choose the side with more open space
        if obs["left"] > obs["right"]:
            micro_path = [
                (x, y + 1, speed, heading),
                (x + 1, y + 1, speed, heading),
                (x + 2, y, speed, heading),
                merge_goal
            ]
        else:
            micro_path = [
                (x, y - 1, speed, heading),
                (x + 1, y - 1, speed, heading),
                (x + 2, y, speed, heading),
                merge_goal
            ]

        return [format_coord(point) for point in micro_path]

    # No obstacle, continue toward macro path
    return [format_coord(merge_goal)]


if __name__ == "__main__":
    client = ArduinoPathClient()

    try:
        while True:
            macro_path = client.get_macro_path()

            if not macro_path:
                print("No macro path received")
                time.sleep(2)
                continue

            print("Macro path:", macro_path)

            current = macro_path[0]

            obs = read_pi_sensors()
            print("Obstacle data:", obs)

            micro_path = generate_micro_path(current, macro_path, obs)
            print("Generated micro path:", micro_path)

            # Send generated micro-path back to GIGA
            if client.send_micro_path(micro_path):
                print("Micro path sent successfully")
            else:
                print("Failed to send micro path")

            time.sleep(5)

    finally:
        client.close()
