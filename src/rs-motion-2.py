import time
import pyrealsense2 as rs
import numpy as np
# import ipdb
i = 0

# rs.log_severity(rs.log_severity.debug)
# rs.log_to_console(rs.log_severity.debug)

gryro_iter = 1
accel_iter = 0
depth_iter = 0
color_iter = 0
ts_gyro = 0.0
ts_accel = 0.0
ts_depth = 0.0
ts_color = 0.0
color_gryo_latency = 0.0
color_flag = False
gryo_domain = ""
accel_domain = ""
depth_domain = ""
color_domain = ""

playback = None
sleep_time = 1.0

def callback(frame):
    global gryro_iter, accel_iter, depth_iter, color_iter, ts_gyro, ts_accel, ts_depth, ts_color, color_gryo_latency, color_flag
    # print(frame.profile.stream_type(), frame.timestamp)
    if frame.profile.stream_type() == rs.stream.color:
        color_iter +=1
        ts_color = frame.timestamp
        if color_iter == 30:
            color_flag = True
    elif frame.profile.stream_type() == rs.stream.gyro:
        gryro_iter += 1
        ts_gyro = frame.timestamp
    elif frame.profile.stream_type() == rs.stream.accel:
        accel_iter += 1
        ts_accel = frame.timestamp
    elif frame.profile.stream_type() == rs.stream.depth:
        depth_iter += 1
        ts_depth = frame.timestamp
    time.sleep(0.001)

def main():
    global gryro_iter, accel_iter, depth_iter, color_iter, ts_gyro, ts_accel, ts_depth, ts_color, color_gryo_latency
    global playback
    config = rs.config()
    pipeline = rs.pipeline()
    rs.config.enable_device_from_file(config, "/media/jeremy/Samsung_T5/jeremy/realsense-tracking/20191201_154627.bag", False)
    
    profile = config.resolve(pipeline)
    dev = profile.get_device()
    playback = dev.as_playback()
    playback.set_real_time(False)


    sensors = playback.sensors

    for sensor in sensors:
        profile = sensor.profiles[0]
        if profile.stream_type() == rs.stream.infrared:
            sensor.open(profile)
            sensor.open(sensor.profiles[1])
        elif profile.stream_type() == rs.stream.color:
            sensor.open(profile)
        elif profile.stream_type() == rs.stream.gyro:
            sensor.open(profile)
            sensor.open(sensor.profiles[1])
        # ipdb.set_trace()
        print(profile)
        sensor.start(callback)
    

    while True:
        print("FPS --- Gyro: {:.1f}; Accel: {:.1f}; Depth: {:.1f}; Color: {:.1f}".format(gryro_iter/sleep_time, accel_iter/sleep_time, depth_iter/sleep_time, color_iter/sleep_time))
        print("Timing --- Gyro: {:.1f}; Accel: {:.1f}; Depth: {:.1f}; Color: {:.1f}".format(ts_gyro, ts_accel, ts_depth, ts_color))
        print("Latency --- ColorToGryo: {}".format(ts_gyro - ts_color))
        print()

        if gryro_iter == 0:
            break
        gryro_iter = 0
        accel_iter = 0
        depth_iter = 0
        color_iter = 0
        time.sleep(sleep_time)

    
if __name__ == "__main__":
    main()