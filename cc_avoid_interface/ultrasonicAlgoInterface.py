import serial
import time
import struct
import math
from enum import StrEnum
from gpiozero import DistanceSensor

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
            self.ser = serial.Serial(
                port,
                baudrate,
                timeout=self.READ_TIMEOUT
            )
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.ser = None
            return None

        time.sleep(self.STARTUP_DELAY)
        self.ser.reset_input_buffer()
    
    def readUltrasonics():
        sensorFlags = [0,0,0]
        sensor1 = DistanceSensor(echo=24, trigger=23, max_distance=4)
        sensor2 = DistanceSensor(echo=6, trigger=5, max_distance=4)
        sensor3 = DistanceSensor(echo=27, trigger=17, max_distance=4)

        if (sensor1.distance * 100) < 400:
            sensorFlags[0] = 1

        if (sensor2.distance * 100) < 400:
            sensorFlags[1] = 1
            
        if (sensor3.distance * 100) < 400:
            sensorFlags[2] = 1
        
        return sensorFlags
    
    def algorithm(self, currPos, nextPos, sensorFlags):
        x1, y1, currVel, currHead = currPos
        x2, y2, nextVel, nextHead = nextPos
        
        dx = x2 - x1
        dy = y2 - y1
        angle = math.degrees(math.atan2(dy, dx))
        
        if (angle >= 0 and angle < 60):
            if (sensorFlags[2] == 1):
                newPos = (x2 + 1), y2, nextVel, nextHead
        elif (angle >= 60 and angle < 120):
            if (sensorFlags[1] == 1):
                newPos = (x2 - 1), y2, nextVel, nextHead
        else:
            if (sensorFlags[0] == 1):
                newPos = (x2 - 1), y2, nextVel, nextHead
                
        self.send_micro_path(newPos)

    # ---------------- MACRO PATH (Arduino → Pi) ----------------
    def get_macro_path(self):

        self.ser.write(f"{Protocol.START}\n".encode())

        start_time = time.time()
        path = []
        in_stream = False

        while time.time() - start_time < self.OPERATION_TIMEOUT:
            if in_stream:
                # Read exactly one Coord worth of bytes
                data = self.ser.read(COORD_SIZE)
                if len(data) < COORD_SIZE:
                    continue


                try:
                    x, y, speed, heading = struct.unpack("<ffff", data)
                    if heading < 0:
                        # heading should never be negative. this is used to signal the end of coord data
                        in_stream = False
                        continue

                    path.append((x, y, speed, heading))
                    print(f"  coord: ({x:.3f}, {y:.3f}, {speed:.3f}, {heading:.3f})")
                except struct.error:
                    print("Unpack error, skipping")
            else:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                print(f"[ctrl] {line}")
                if line == Protocol.ACK_START:
                    in_stream = True
                elif line == Protocol.END_STREAM:   # "DONE_STREAM"
                    break

        return path

    # ---------------- MICRO PATH (Pi → Arduino) ----------------
    def send_micro_path(self, coord):
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
        if self.ser == None:
            return
            
        if self.ser.is_open:
            self.ser.close()


# ---------------- TEST ----------------
if __name__ == "__main__":
    client = ArduinoPathClient()

    try:
        while True:
            path = client.get_macro_path()
            sensorFlags = client.readUltrasonics()
            
            if 1 in sensorFlags:
                currPos, nextPos = client.get_macro_path()
                
            
            print("PATH:", path if path else "no data")

            time.sleep(2)

            micro_p = [
                (0, 1, 7.5, 10.2),
                (2.08, 10, 5, 12),
                (4.7, 13.105, 2, 0),
                (8.4, 13.3, 10, 0.112),
            ]

            for coord in micro_p:
                ok = client.send_micro_path(coord)
                if not ok:
                    print("Failed sending:", coord)

            time.sleep(2)

    except Exception as e:
        print("Error:", e)

    finally:
        client.close()