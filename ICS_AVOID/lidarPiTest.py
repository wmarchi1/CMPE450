import time
import math
import struct
import serial
from enum import StrEnum

# pip install rplidar-roboticia
from rplidar import RPLidar


COORD_SIZE = 16


class Protocol(StrEnum):
    START = "Start"
    READY = "Ready"
    INITIATE = "Initiate"
    DONE = "Done"
    END_STREAM = "DONE_STREAM"
    ACK_START = "ACK_START"
    ACK_READY = "ACK_READY"


class ArduinoPathClient:
    def __init__(
        self,
        port="/dev/ttyACM0",
        baudrate=115200,
        read_timeout=0.2,
        operation_timeout=2.0,
        startup_delay=2.0,
    ):
        self.READ_TIMEOUT = read_timeout
        self.OPERATION_TIMEOUT = operation_timeout
        self.STARTUP_DELAY = startup_delay

        try:
            self.ser = serial.Serial(port, baudrate, timeout=self.READ_TIMEOUT)
        except serial.SerialException as e:
            print(f"Error opening Arduino serial port: {e}")
            self.ser = None
            return

        time.sleep(self.STARTUP_DELAY)
        self.ser.reset_input_buffer()

    def get_macro_path(self):
        self.ser.write(f"{Protocol.START}\n".encode())

        start_time = time.time()
        path = []
        in_stream = False

        while time.time() - start_time < self.OPERATION_TIMEOUT:
            if in_stream:
                data = self.ser.read(COORD_SIZE)
                if len(data) < COORD_SIZE:
                    continue

                try:
                    x, y, speed, heading = struct.unpack("<ffff", data)
                    if heading < 0:
                        in_stream = False
                        continue

                    path.append((x, y, speed, heading))
                except struct.error:
                    print("Unpack error, skipping")
            else:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                if line == Protocol.ACK_START:
                    in_stream = True
                elif line == Protocol.END_STREAM:
                    break

        return path

    def send_micro_path(self, coord):
        if self.ser is None:
            return False

        self.ser.write(f"{Protocol.READY}\n".encode())
        start_time = time.time()

        while time.time() - start_time < self.OPERATION_TIMEOUT:
            line = self.ser.readline().decode(errors="ignore").strip()

            if not line:
                continue

            if line == Protocol.INITIATE:
                packet = struct.pack("<ffff", *coord)
                self.ser.write(packet)

            elif line == Protocol.DONE:
                return True

        return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


class ObstacleAvoider:
    def __init__(
        self,
        lidar_port="/dev/ttyUSB0",
        arduino_port="/dev/ttyACM0",
        safe_distance_mm=650,
        danger_distance_mm=350,
        forward_speed=5.0,
        turn_speed=3.0,
        waypoint_distance_m=0.75,
    ):
        self.lidar = RPLidar(lidar_port, baudrate=115200)
        self.arduino = ArduinoPathClient(port=arduino_port)

        self.safe_distance_mm = safe_distance_mm
        self.danger_distance_mm = danger_distance_mm
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.waypoint_distance_m = waypoint_distance_m

    @staticmethod
    def normalize_angle(angle):
        angle = angle % 360
        if angle > 180:
            angle -= 360
        return angle

    @staticmethod
    def angle_to_xy(distance_m, heading_deg):
        rad = math.radians(heading_deg)
        x = distance_m * math.sin(rad)
        y = distance_m * math.cos(rad)
        return x, y

    def scan_to_bins(self, scan):
        """
        Converts lidar scan to 10-degree bins.

        RPLidar scan item format:
            quality, angle_degrees, distance_mm
        """
        bins = {a: [] for a in range(-180, 181, 10)}

        for quality, angle, distance in scan:
            if quality == 0 or distance <= 0:
                continue

            heading = self.normalize_angle(angle)

            nearest_bin = round(heading / 10) * 10
            nearest_bin = max(-180, min(180, nearest_bin))

            bins[nearest_bin].append(distance)

        # Use median-ish minimum filtering for safety.
        processed = {}
        for angle, values in bins.items():
            if values:
                processed[angle] = min(values)
            else:
                processed[angle] = None

        return processed

    def choose_heading(self, bins):
        """
        Chooses the safest heading.

        Assumption:
        0 degrees = forward
        positive = right
        negative = left

        Adjust this if your lidar/robot coordinate frame is different.
        """
        candidate_angles = list(range(-90, 91, 10))

        best_angle = 0
        best_score = -1

        for angle in candidate_angles:
            distance = bins.get(angle)

            if distance is None:
                distance = 12000  # Treat unknown as open, but not perfect.

            # Prefer open space, but penalize sharp turns.
            clearance_score = min(distance, 2500)
            forward_bias = 1000 - abs(angle) * 8
            score = clearance_score + forward_bias

            if score > best_score:
                best_score = score
                best_angle = angle

        front_distances = [
            bins.get(a) for a in range(-20, 21, 10)
            if bins.get(a) is not None
        ]

        front_min = min(front_distances) if front_distances else 12000

        if front_min < self.danger_distance_mm:
            # Emergency turn toward the more open side.
            left = min([bins.get(a, 12000) or 12000 for a in range(-90, -20, 10)])
            right = min([bins.get(a, 12000) or 12000 for a in range(20, 91, 10)])
            return -75 if left > right else 75, self.turn_speed

        if front_min < self.safe_distance_mm:
            return best_angle, self.turn_speed

        return 0, self.forward_speed

    def make_micro_coord(self, heading_deg, speed):
        x, y = self.angle_to_xy(self.waypoint_distance_m, heading_deg)

        # coord format expected by Arduino:
        # x, y, speed, heading
        return (float(x), float(y), float(speed), float(heading_deg))

    def run(self):
        try:
            print("Starting obstacle avoidance...")

            for scan in self.lidar.iter_scans():
                bins = self.scan_to_bins(scan)
                heading, speed = self.choose_heading(bins)
                coord = self.make_micro_coord(heading, speed)

                print(f"Sending avoidance waypoint: {coord}")

                ok = self.arduino.send_micro_path(coord)
                if not ok:
                    print("Arduino did not confirm waypoint.")

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("Stopped by user.")

        finally:
            self.lidar.stop()
            self.lidar.disconnect()
            self.arduino.close()


if __name__ == "__main__":
    avoider = ObstacleAvoider(
        lidar_port="/dev/ttyUSB0",
        arduino_port="/dev/ttyACM0",
        safe_distance_mm=650,
        danger_distance_mm=350,
    )

    avoider.run()
