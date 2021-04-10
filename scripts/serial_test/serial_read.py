import serial
import time

def main():
    msg = b"Hello"
    msg_bytes = len(msg)
    update_rate = 1
    interval  = 1.0 / update_rate
    with serial.Serial('/dev/ttyUSB0', 19200, timeout=1) as ser:  # open serial port
        print(ser.name)         # check which port was really used
        print("Opening up Serial Port")
        while (True):
            time.sleep(interval)
            print(f"Attempting to read {msg_bytes} bytes")
            msg_received = ser.read(msg_bytes)
            print(f"Received Message: {msg_received}")
        ser.close()             # close port

if __name__ == "__main__":
    main()