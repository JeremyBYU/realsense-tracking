import serial
import time

def main():
    msg = "Hello\n"
    msg_bytes =  msg.encode('utf-8')
    update_rate = 1
    interval  = 1.0 / update_rate
    with serial.Serial('/dev/ttyUSB0', 19200, timeout=1) as ser:  # open serial port
        print(ser.name)         # check which port was really used
        print("Opening up Serial Port")
        while (True):
            time.sleep(interval)
            ser.write(msg_bytes)
            print(f"Wrote Message that is {len(msg_bytes)} bytes")
        ser.close()             # close port

if __name__ == "__main__":
    main()