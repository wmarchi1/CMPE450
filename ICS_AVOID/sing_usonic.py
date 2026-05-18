# import lgpio as GPIO
# import time

# TRIG = 23
# ECHO = 24

# h = GPIO.gpiochip_open(0)

# def setup():
#     GPIO.gpio_claim_output(h, TRIG)
#     GPIO.gpio_claim_input(h, ECHO)

# def get_distance():
#     GPIO.gpio_write(h, TRIG, 0)
#     time.sleep(0.05)

#     GPIO.gpio_write(h, TRIG, 1)
#     time.sleep(0.00001)
#     GPIO.gpio_write(h, TRIG, 0)

#     pulse_start = time.time()
#     timeout = pulse_start + 1

#     while GPIO.gpio_read(h, ECHO) == 0:
#         pulse_start = time.time()
#         if pulse_start > timeout:
#             return None

#     pulse_end = time.time()
#     timeout = pulse_end + 1

#     while GPIO.gpio_read(h, ECHO) == 1:
#         pulse_end = time.time()
#         if pulse_end > timeout:
#             return None

#     pulse_duration = pulse_end - pulse_start
#     distance = pulse_duration * 17150
#     return round(distance, 2)

# def cleanup():
#     try:
#         GPIO.gpio_free(h, TRIG)
#     except:
#         pass

#     try:
#         GPIO.gpio_free(h, ECHO)
#     except:
#         pass

#     GPIO.gpiochip_close(h)

# if __name__ == '__main__':
#     try:
#         setup()

#         while True:
#             dist = get_distance()
#             if dist is None:
#                 print("Timeout: no reading")
#             else:
#                 print("Measured Distance = {:.2f} cm".format(dist))
#             time.sleep(1)

#     except KeyboardInterrupt:
#         print("Measurement stopped by user")

#     finally:
#         cleanup()

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
