import serial
import time
import argparse
import ctypes

from .common import MessagePoseUpdate, MessageLandingCommand, get_us_from_epoch, serialize

def main(serial_port, baud_rate, freq):
    now = get_us_from_epoch()
    msg_pose_update = MessagePoseUpdate(pose_update=(now, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0))
    msg_landing_command = MessageLandingCommand(landing_command=(now, 0.0, 1.0, 2.0))

    interval  = 1.0 / freq
    print(f"Opening up Serial Port {serial_port} with baud rate of {baud_rate}")
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser:  # open serial port
        while (True):
            time.sleep(interval)
            now = get_us_from_epoch()
            msg_pose_update.pose_update.time_us = now
            msg_landing_command.landing_command.time_us = now
            # msg_serialized = msg_pose_update.get_bytes_array() # this will work, but makes a copy
            msg_serialized = serialize(msg_pose_update).contents.raw # This makes no copy, preferred
            ser.write(msg_serialized)
            # Also write landing command
            msg_serialized = serialize(msg_landing_command).contents.raw
            ser.write(msg_serialized)
            print(f"Wrote Message that is {ctypes.sizeof(msg_pose_update)} bytes")
        ser.close()             # close port

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