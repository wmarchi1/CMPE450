import serial
import time

TIME_OUT = 0.1 # 100ms
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2) 
ser.reset_input_buffer()

def get_giga_data(ser): 
    
    # initiate a request to the arduino
    ser.write(b"Start\n")
    start = time.time()
    path = []
    
    
    while True:
        if time.time() - start > TIME_OUT: # everything needs to be sent within this period
            break

        line = ser.readline().decode('utf-8').strip() # expect to read (x, y, theta)
        if line == "END":
            break
        else:
            try:
                x, y, theta_h = map(int, line[1:-1].split(","))
                path.append((x, y, theta_h))
            except:
                pass # something went wrong with the map function (line wasn't in the correct format)
            
    return path

if __name__ == "__main__": 
    while True:
        path = get_giga_data(ser)
        if path:
            print(path)

        time.sleep(5)
