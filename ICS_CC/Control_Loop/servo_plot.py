import serial
import matplotlib.pyplot as plt
import csv

ser = serial.Serial('COM21', 115200)  # change COM port

heading_data = []
velocity_data = []
pwm_data = []
servo_data = []

plt.ion()
fig, ax = plt.subplots()

# open CSV file
with open("data_log.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Heading", "Velocity", "PWM", "Servo Angle"])

    try:
        while True:
            line = ser.readline().decode().strip()
            parts = line.split(",")

            if len(parts) == 4:
                try:
                    heading = float(parts[0])
                    velocity = float(parts[1])
                    pwm = float(parts[2])
                    servo = float(parts[3])

                    heading_data.append(heading)
                    velocity_data.append(velocity)
                    pwm_data.append(pwm)
                    servo_data.append(servo)

                    # write to CSV
                    writer.writerow([heading, velocity, pwm, servo])

                    # plot heading vs velocity
                    ax.clear()
                    ax.plot(heading_data, velocity_data)
                    ax.set_xlabel("Heading")
                    ax.set_ylabel("Velocity")
                    ax.grid()

                    plt.pause(0.01)

                except ValueError:
                    print("Bad data:", line)

    except KeyboardInterrupt:
        print("\nStopping cleanly...")
        ser.close()
