"""Script to parse a saved bag file and extract images into a folder and IMU data into CSV files
This was useful as creating the correct form of input for XIVO. However, not using that anymore.
"""

import time
import argparse
import pathlib
import csv

import numpy as np
import pyrealsense2 as rs
import cv2
# import ipdb

MS_TO_NS = 1000000
sleep_time = 0.001

DEFAULT_SENSORS = ['infrared1', 'gyro', 'accel']
SENSOR_MAPPING = dict(infrared1=rs.stream.infrared, color=rs.stream.color,
                      gyro=rs.stream.gyro, accel=rs.stream.accel, depth=rs.stream.depth)

SENSOR_MAPPING_DIR = dict(infrared1='cam0', color='cam0',
                          gyro='imu0', accel='imu0', depth='stereo')

DIR_CSV_HEADINGS = dict(cam0=['timestamp', 'filename'],
                        imu0=['timestamp', 'omega_x', 'omega_y', 'omega_z', 'alpha_x', 'alpha_y', 'alpha_z'])


# global variable for handling conversion and saving, will be of class Converter
CONVERTER = None
np.set_printoptions(suppress=True,
   formatter={'float_kind':'{:0.2f}'.format, 'int_kind': '{:d}'})  #float, 2 units 


class Converter(object):
    def __init__(self, data_dir="data", bag_name='rsbag', sensors=DEFAULT_SENSORS, image_subdir='data', csv_fname='data.csv'):
        self.allowed_rs_sensors = [SENSOR_MAPPING[sensor]
                                   for sensor in sensors]
        # Create data directory
        self.image_subdir = image_subdir
        self.data_dir = pathlib.Path(data_dir) / bag_name
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.csv_fname = csv_fname

        # create sensor subdir
        self.sensor_dirs = dict()
        self.sensor_dirs_alt = dict()
        self.sensor_data = dict()
        for sensor in sensors:
            # create and save subdirectory for saving sensor data
            self.sensor_dirs[sensor] = self.data_dir / \
                pathlib.Path(SENSOR_MAPPING_DIR[sensor])
            self.sensor_dirs_alt[SENSOR_MAPPING_DIR[sensor]
                                 ] = self.sensor_dirs[sensor]
            self.sensor_dirs[sensor].mkdir(parents=True, exist_ok=True)
            # create empty data stucture for recording filenames/data (later saved as csv file)
            self.sensor_data[SENSOR_MAPPING_DIR[sensor]] = list()
            # Unfortunately XIVO requires the actual camera images to be stored in a subdirectory call data
            if SENSOR_MAPPING_DIR[sensor] == 'cam0':
                (self.sensor_dirs[sensor] /
                 'data').mkdir(parents=True, exist_ok=True)

    def handle_image(self, sensor_name, frame):
        data_repo = self.sensor_data[SENSOR_MAPPING_DIR[sensor_name]]
        data_dir = self.sensor_dirs[sensor_name]

        vframe = frame.as_video_frame()
        ts = int(frame.timestamp * MS_TO_NS)
        image_data = np.asanyarray(vframe.get_data())

        data_dir = data_dir / self.image_subdir
        fname = sensor_name + '_' + str(ts) + '.png'
        fpath = data_dir / fname

        # write file
        cv2.imwrite(str(fpath), image_data)
        # record data in list of dicts
        data_repo.append(dict(timestamp=ts, filename=fname))

    def handle_imu(self, sensor_name, frame):
        data = frame.as_motion_frame().get_motion_data()
        ts = int(frame.timestamp * MS_TO_NS)

        data_repo = self.sensor_data[SENSOR_MAPPING_DIR[sensor_name]]
        data_repo.append(dict(x=data.x, y=data.y, z=data.z,
                              sensor=sensor_name, timestamp=ts))

    def save_csv(self):
        for sensor_dir_name, sensor_repo in self.sensor_data.items():
            data_dir = self.sensor_dirs_alt[sensor_dir_name]
            fpath = data_dir / self.csv_fname
            print(sensor_dir_name, data_dir)
            if sensor_dir_name == 'imu0':
                new_sensor_repo = self.organize_imu()
                self.write_csv(sensor_dir_name, str(fpath), new_sensor_repo)
            else:
                self.write_csv(sensor_dir_name, str(fpath), sensor_repo)


    def convert_numpy_imu(self, data_repo):
        accel_np = np.array([[data['timestamp'], data['x'], data['y'], data['z']] for data in data_repo if data['sensor'] == 'accel'])
        gyro_np = np.array([[data['timestamp'], data['x'], data['y'], data['z']] for data in data_repo if data['sensor'] == 'gyro'])
        # print(accel_np.shape)
        # print(gyro_np.shape)
        return accel_np, gyro_np

    def accel_is_slower(self, accel_np, gyro_np):
        return accel_np.shape[0] < gyro_np.shape[0]

    def linear_interoplate(self, slow_stream, fast_stream):
        x_input = slow_stream[:, 0]
        x_model = fast_stream[:, 0]
        new_fast_stream = [x_input]
        for i in [1,2,3]: # x,y,z axis of fast_sream
            y_model = fast_stream[:, i]
            y_predict = np.interp(x_input, x_model, y_model)
            new_fast_stream.append(y_predict)
        new_fast_stream = np.column_stack(new_fast_stream)
        return new_fast_stream
    
    def organize_imu(self):
        data_repo = self.sensor_data['imu0']
        accel_np, gyro_np = self.convert_numpy_imu(data_repo)

        sync_with_accel = self.accel_is_slower(accel_np, gyro_np)
        slow_stream, fast_stream = (accel_np, gyro_np) if sync_with_accel else (gyro_np, accel_np)
        # ensure the data is sorted
        slow_stream = slow_stream[slow_stream[:,0].argsort()]
        fast_stream = fast_stream[fast_stream[:,0].argsort()]

        new_fast_stream = self.linear_interoplate(slow_stream, fast_stream)


        combined = np.column_stack((slow_stream, new_fast_stream[:, 1:]))
        new_data_repo = []
        for i in range(combined.shape[0]):
            dt = combined[i, :]
            if sync_with_accel:
                new_data = dict(timestamp=int(dt[0]), alpha_x=dt[1],alpha_y=dt[2],alpha_z=dt[3], omega_x=dt[4], omega_y=dt[5], omega_z=dt[6])
            else:
                new_data = dict(timestamp=int(dt[0]), alpha_x=dt[4],alpha_y=dt[5],alpha_z=dt[6], omega_x=dt[1], omega_y=dt[2], omega_z=dt[3])
            new_data_repo.append(new_data)

        # print(slow_stream[15:20, :])
        # print(fast_stream[15:20,:])
        # print(new_fast_stream[15:20, :])
        return new_data_repo


    def write_csv(self, sensor_dir_name, fpath, records):

        csv_columns = DIR_CSV_HEADINGS[sensor_dir_name]
        with open(fpath, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=csv_columns)
            writer.writeheader()
            for data in records:
                writer.writerow(data)


class DefaultListSensors(argparse.Action):
    CHOICES = ['infrared1', 'gyro', 'accel', 'color']

    def __call__(self, parser, namespace, values, option_string=None):
        if values:
            for value in values:
                if value not in self.CHOICES:
                    message = ("invalid choice: {0!r} (choose from {1})"
                               .format(value,
                                       ', '.join([repr(action)
                                                  for action in self.CHOICES])))

                    raise argparse.ArgumentError(self, message)
            setattr(namespace, self.dest, values)


def callback(frame):
    # print(frame.profile.stream_type(), frame.timestamp)
    if frame.profile.stream_type() == rs.stream.infrared:
        # print('infrared1')
        CONVERTER.handle_image('infrared1', frame)
        pass
    elif frame.profile.stream_type() == rs.stream.color:
        # print('color')
        pass
    elif frame.profile.stream_type() == rs.stream.gyro:
        CONVERTER.handle_imu('gyro', frame)
    elif frame.profile.stream_type() == rs.stream.accel:
        CONVERTER.handle_imu('accel', frame)
        pass
    elif frame.profile.stream_type() == rs.stream.depth:
        # print('depth')
        pass
    time.sleep(0.001)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert RS Data'
    )
    parser.add_argument('--bag', '-b', action='store',
                        required=True,
                        help="Path to bag file")
    parser.add_argument('--data_dir', '-d', action="store",
                        default="data",
                        help='The directory to save data',
                        )
    parser.add_argument('--sensors', '-s', nargs='*', action=DefaultListSensors,
                        default=DEFAULT_SENSORS,
                        help='Sensors to read from bag file and convert  and save',
                        metavar='SENSORS')
    parser.add_argument('--image_subdir', '-isd', action='store',
                        default='data',
                        help="Subdirectory to store image files in")

    args = parser.parse_args()
    print(args)
    return args


def main():
    global CONVERTER
    args = parse_args()
    bag_file = pathlib.Path(args.bag)
    CONVERTER = Converter(args.data_dir, bag_file.stem, sensors=args.sensors,
                          image_subdir=args.image_subdir)

    config = rs.config()
    pipeline = rs.pipeline()
    rs.config.enable_device_from_file(config, args.bag, False)

    profile = config.resolve(pipeline)
    dev = profile.get_device()
    playback = dev.as_playback()
    playback.set_real_time(False)

    sensors = playback.sensors

    for sensor in sensors:
        profiles = sensor.profiles
        if profiles:
            sensor.open(profiles)
            sensor.start(callback)

    while True:
        # print(dir(playback))
        time.sleep(0.1)
        status = playback.current_status()
        # print(status)
        if status == rs.playback_status.stopped:
            break

    # Now save the csv file
    CONVERTER.save_csv()


if __name__ == "__main__":
    main()
