from gpiozero import DistanceSensor
from time import sleep

# Trigger on GPIO 23, Echo on GPIO 24
sensor1 = DistanceSensor(echo=24, trigger=23, max_distance=4)
sensor2 = DistanceSensor(echo=6, trigger=5, max_distance=4)
sensor3 = DistanceSensor(echo=27, trigger=17, max_distance=4)


while True:
    print(f"Sensor 1 Distance: {sensor1.distance * 100:.1f} cm")
    print(f"Sensor 2 Distance: {sensor2.distance * 100:.1f} cm")
    print(f"Sensor 3 Distance: {sensor3.distance * 100:.1f} cm")
    print("")
    sleep(0.5)
