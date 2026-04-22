import serial
import time

class ArduinoPathClient:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=0.6, startup_delay=2):
        self.TIME_OUT = timeout
        self.ser = serial.Serial(port, baudrate, timeout=self.TIME_OUT)
        time.sleep(startup_delay)
        self.ser.reset_input_buffer()

    def get_macro_path(self):
        self.ser.write(b"Start\n")
        start = time.time()
        path = []

        while True:
            if time.time() - start > self.TIME_OUT:
                break

            line = self.ser.readline().decode('utf-8').strip()

            if line == "END":
                break
            else:
                try:
                    x, y, theta_h = map(int, line[1:-1].split(","))
                    path.append((x, y, theta_h))
                except Exception:
                    pass

        return path

    def send_micro_path(self, micro_p=None):
        if micro_p is None:
            micro_p = [
                "(0, 1, 50)",
                "(10, 17, 40)",
                "(80, 12, 0)",
                "(85, 20, 15)",
                "(90, 10, 0)"
            ]

        self.ser.write(b"Ready\n")
        start = time.time()

        while True:
            if time.time() - start > self.TIME_OUT:
                return False

            line = self.ser.readline().decode('utf-8').strip()

            if line == "Initiate":
                data = str(len(micro_p)) + "\n" + "\n".join(micro_p) + "\n"
                self.ser.write(data.encode("utf-8"))
            elif line == "Done":
                return True
            elif line == "Incomplete":
                return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


if __name__ == "__main__":
    client = ArduinoPathClient()

    try:
        while True:
            path = client.get_macro_path()
            if path:
                print(path)

            time.sleep(5)

            if client.send_micro_path():
                print("success")

            time.sleep(5)

    finally:
        client.close()
