import serial
import time

def main():
    update_rate = 1
    interval  = 1.0 / update_rate
    with serial.Serial('/dev/ttyS4', 19200, timeout=1) as ser:  # open serial port
        print(ser.name)         # check which port was really used
        print("Opening up Serial Port")
        while (True):
            time.sleep(interval)
            print(f"Attempting to read")
            msg_bytes = ser.readline())
            msg_received = msg_bytes.decode('utf-8')
            print(f"Received Message: {msg_received}")
        ser.close()             # close port

if __name__ == "__main__":
    main()