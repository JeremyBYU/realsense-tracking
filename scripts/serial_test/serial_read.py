import serial
import time
import argparse
import ctypes

from .common import PoseUpdate, deserialize

def main(serial_port, baud_rate, freq):
    pose_update = PoseUpdate()
    num_bytes_to_read = ctypes.sizeof(pose_update)
    interval  = 1.0 / freq
    print(f"Opening up Serial Port {serial_port} with baud rate of {baud_rate}")
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser:  # open serial port
        while (True):
            time.sleep(interval)
            msg_serialized = ser.read(num_bytes_to_read)
            deserialize(pose_update, msg_serialized)
            print(f"Received Pose: {pose_update}")

def parse_args():
    parser = argparse.ArgumentParser(description='Serial Tester')
    parser.add_argument('--serial', type=str, help='Serial Port, e.g. /dev/ttyS4', default="/dev/ttyS4")
    parser.add_argument('--baud', type=int, help="Baud rate,e.g, 19200", default=19200)
    parser.add_argument('--freq', type=int, help="Frequency", default=1)
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    main(args.serial, args.baud, args.freq)
