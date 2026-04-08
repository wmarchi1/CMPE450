import serial
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/cu.usbmodem1051DB2D76602', 115200)

x_data = []
y_data = []

plt.ion()
fig, ax = plt.subplots()

try:
    while True:
        line = ser.readline().decode().strip()
        parts = line.split(",")

        if len(parts) == 2:
            x = float(parts[0])
            y = float(parts[1])

            x_data.append(x)
            y_data.append(y)

            ax.clear()
            ax.plot(x_data, y_data)
            ax.set_aspect('equal', adjustable='box')
            ax.grid()

            plt.pause(0.01)

except KeyboardInterrupt:
    print("\nStopping cleanly...")
    ser.close()
