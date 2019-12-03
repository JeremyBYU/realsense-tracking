import time
import pyrealsense2 as rs
import numpy as np
i = 0
def callback(frame):
    print(frame.profile.stream_type(), frame.timestamp)
    i +=1

def main():
    config = rs.config()
    pipeline = rs.pipeline()
    # config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)
    config.enable_stream(rs.stream.color)

    rs.config.enable_device_from_file(config, "/media/jeremy/Samsung_T5/jeremy/realsense-tracking/20191201_154627.bag", False)
    profile = pipeline.start(config)
    # profile = pipeline.start(config, callback)
    playback=profile.get_device().as_playback()
    playback.set_real_time(False)

    # while True:
    #     time.sleep(1)


    i = 0
    while True:
        sucess, frames = pipeline.try_wait_for_frames(1)
        if sucess and frames:
            print("")
        else:
            continue
        for frame in frames:
            print(frame.profile.stream_type(), frame.timestamp)
            
            # if frame.is_motion_frame():
            #     print("Frame ts: : {}".format(frame.get_timestamp()))
        i += 1
        print("")
if __name__ == "__main__":
    main()