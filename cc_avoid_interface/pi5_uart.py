import serial
import time

TIME_OUT = 0.6 # 600ms
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=TIME_OUT)
time.sleep(2) 
ser.reset_input_buffer()

def get_macro_path(ser): 
    
    # initiate a request to the arduino giga
    ser.write(b"Start\n")
    start = time.time()
    path = []
    # print("here")
    
    
    while True:
        if time.time() - start > TIME_OUT: # everything needs to be sent within this period
            break

        line = ser.readline().decode('utf-8').strip() # expect to read (x, y, theta)
        if line == "END":
            break
        else:
            # print("got here")
            try:
                x, y, theta_h = map(int, line[1:-1].split(","))
                path.append((x, y, theta_h))
                # path.append(line)
            except:
                pass # something went wrong with the map function (line wasn't in the correct format)
            
    # print("done")
    return path

def send_micro_path(ser):
    micro_p = ["(0, 1, 50)", "(10, 17, 40)", "(80, 12, 0)", "(85, 20, 15)", "(90, 10, 0)"]

    # lets the arduino giga know that it's ready
    ser.write(b"Ready\n")
    start = time.time()

    while True:

        if time.time() - start > TIME_OUT:
            return False

        line = ser.readline().decode('utf-8').strip()
        if line == "Initiate":
            data = str(len(micro_p)) + "\n" + "\n".join(micro_p) + "\n"
            ser.write(data.encode("utf-8"))
        elif line == "Done":
            return True
        elif line == "Incomplete":
            return False



if __name__ == "__main__": 
    while True:
        path = get_macro_path(ser)
        if path:
            print(path)

        time.sleep(5)

        if send_micro_path(ser):
            print("success")

        time.sleep(5)
