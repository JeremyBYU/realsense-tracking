// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
// Realsense Tracking Module

// This technique works with RSUSB driver (LIBUVC) but should work with kernel drivers as well
// This example is meant for D435i, to get high frequency motion data at the same time of depth,rgb images
// Two examples are shown, one that read a bags file and another that does live streaming from the sensor. Activate bag reading with cmd line flag --bag=<file>
// The main technique that makes this possible is to NOT USE rs::pipeline, but to directly acess each sensor and 
// configure the stream manually (as well as associated callbacks).

#include <cstring>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <string>
#include <sstream>
#include <list>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <thread>

#include <librealsense2/rs.hpp>
#include <gflags/gflags.h>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>

#include "utility.hpp"
#include "ImageData.pb.h"
#include "IMUMessage.pb.h"

using namespace std::chrono_literals;

// Command line flags
DEFINE_string(bag, "",
				"Path to bag file");

namespace rstracker
{

// This global variable holds history for collected IMU data
// TODO - Add mutex locking?
static IMUHistory imu_hist(2);

// The slow_sensor has the lower fps/frequency
// This function should be immediatly called AFTER a measurement update is given for the slow_sensor
// This measurement update is stored inside global variable imu_hist
double IMUData_LinearInterpolation(const sensor_name slow_sensor, rstracker_pb::IMUMessage& imu_msg)
{
    sensor_name fast_sensor(static_cast<sensor_name>(!slow_sensor));
    
    // Must have 2 measurements for each sensor
    if (!imu_hist.is_data_full(slow_sensor) || !imu_hist.is_data_full(fast_sensor))
        return -1;
    
    IMUData slow_sensor_recent = imu_hist.recent_data(slow_sensor);
    IMUData slow_sensor_old = imu_hist.old_data(slow_sensor);

    IMUData fast_sensor_recent = imu_hist.recent_data(fast_sensor);
    
    if(fast_sensor_recent.ts < slow_sensor_old.ts)
    {
        // this should not happen. The fast sensor should have a more recent timestamp than the old timestamp for the slow sensor
        std::cout<< "Fast sensor has an older timestamp than the old timestamp from the slow sensor!" << std::endl;
        return -1;
    }
    if(fast_sensor_recent.ts > slow_sensor_recent.ts)
    {
        // this should not happen but might. If this happens we would just need to switch 
        // the fast sensor and the slow sensor, for now just log warning
        std::cout<< "Fast sensor has a newer timestamp than the most recent slow sensor!" << std::endl;
        return -1;
    }
    // Perform interpolation for the slow sensor
    double united_imu_ts = fast_sensor_recent.ts;
    double factor =  (united_imu_ts - slow_sensor_old.ts) / (slow_sensor_recent.ts - slow_sensor_old.ts);
    Float3 slow_interp = slow_sensor_old.data * (1-factor) + slow_sensor_recent.data * factor;

    if (slow_sensor == mACCEL)
    {
		auto gyro_vec = imu_msg.mutable_gyro();
		gyro_vec->set_x(fast_sensor_recent.data.x);
		gyro_vec->set_y(fast_sensor_recent.data.y);
		gyro_vec->set_z(fast_sensor_recent.data.z);

		auto accel_vec = imu_msg.mutable_accel();
		accel_vec->set_x(slow_interp.x);
		accel_vec->set_y(slow_interp.y);
		accel_vec->set_z(slow_interp.z);

		imu_msg.set_ts(united_imu_ts);
    }
    else
    {
		auto gyro_vec = imu_msg.mutable_gyro();
		gyro_vec->set_x(slow_interp.x);
		gyro_vec->set_y(slow_interp.y);
		gyro_vec->set_z(slow_interp.z);

		auto accel_vec = imu_msg.mutable_accel();
		accel_vec->set_x(fast_sensor_recent.data.x);
		accel_vec->set_y(fast_sensor_recent.data.y);
		accel_vec->set_z(fast_sensor_recent.data.z);

		imu_msg.set_ts(united_imu_ts);
    }
    
    return united_imu_ts;
}





int read_bag(std::string file_name)
{
		// create a publisher (topic name "person")
	std::cout << "Read bag" << std::endl;
	eCAL::Initialize(0, nullptr, "RSTrackerPub");
	eCAL::protobuf::CPublisher<rstracker_pb::ImageData> pub_image("ImageData");
	eCAL::protobuf::CPublisher<rstracker_pb::IMUMessage> pub_imu("IMUMessage");
	std::cout << "Make Publisher" << std::endl;

	rs2::config config;
	rs2::device device;
	rs2::pipeline pipe;

	// enable file playback with playback repeat disabled
	config.enable_device_from_file(file_name, false);

	auto profile = config.resolve(pipe);
    device = profile.get_device();
    auto playback = device.as<rs2::playback>();
    playback.set_real_time(false);

	


	auto streams = profile.get_streams();
	auto accel_stream = std::find_if(streams.begin(), streams.end(),
									[](const rs2::stream_profile& x) { return x.stream_type() == RS2_STREAM_ACCEL;});
	auto gryo_stream = std::find_if(streams.begin(), streams.end(),
									[](const rs2::stream_profile& x) { return x.stream_type() == RS2_STREAM_GYRO;});
	if (accel_stream == streams.end() || gryo_stream == streams.end())
	{
		// This bag file doesn't have any gryo and accelerometer data! Exit early.
		std::cout << "This bag file does not have any gyro or accel data. Exiting";
		return EXIT_FAILURE;
	}

	// Which sensor has the lower frame rate.
	bool sync_with_accel = accel_stream->fps() <= gryo_stream->fps();
	auto sensors = playback.query_sensors();

	for (auto &sensor : sensors)
	{
		auto profiles = sensor.get_stream_profiles();
		for(auto &profile: profiles)
		{
			sensor.open(profile);
			std::cout << "Profile: " << profile.stream_name() <<std::endl;
		}
		// Sensor Callback
		sensor.start([&](rs2::frame frame){
			if (frame.get_profile().stream_type() == RS2_STREAM_COLOR)
			{
				const int w = frame.as<rs2::video_frame>().get_width();
        		const int h = frame.as<rs2::video_frame>().get_height();
				const char* image_data = static_cast<const char*>(frame.as<rs2::video_frame>().get_data());

				rstracker_pb::ImageData frame_image;
				frame_image.set_hardware_ts(frame.get_timestamp());
				frame_image.set_width(w);
				frame_image.set_height(h);
				frame_image.set_image_data(image_data);
				pub_image.Send(frame_image);
				
			}
			else if(frame.get_profile().stream_type() == RS2_STREAM_GYRO)
			{
                rs2_vector gyro_sample = frame.as<rs2::motion_frame>().get_motion_data();
                IMUData data({gyro_sample.x, gyro_sample.y, gyro_sample.z}, frame.get_timestamp());
                imu_hist.add_data(mGYRO, data);
                if (!sync_with_accel)
                {
                    rstracker_pb::IMUMessage msg;
                    double ts_ = IMUData_LinearInterpolation(mGYRO, msg);
                    if (ts_ > 0)
                    {
                        // no issues with linear interpolation and the msg
                        print_message(msg, imu_hist);
						pub_imu.Send(msg);
                    }
                }

			}
			else if(frame.get_profile().stream_type() == RS2_STREAM_ACCEL)
			{
                rs2_vector accel_sample = frame.as<rs2::motion_frame>().get_motion_data();
                IMUData data({accel_sample.x, accel_sample.y, accel_sample.z}, frame.get_timestamp());
                imu_hist.add_data(mACCEL, data);
                if (sync_with_accel)
                {
                    rstracker_pb::IMUMessage msg;
                    double ts_ = IMUData_LinearInterpolation(mACCEL, msg);
                    if (ts_ > 0)
                    {
                        // no issues with linear interpolation and the msg
                        // print_message(msg, imu_hist);
						pub_imu.Send(msg);
                    }
                }

			}
			else if(frame.get_profile().stream_type() == RS2_STREAM_DEPTH)
			{

			}
			std::this_thread::sleep_for( 1ms );

		});
	}

    while(true)
    {
        std::this_thread::sleep_for( 1000ms );
    }
	return EXIT_SUCCESS;
}


void print_profiles(std::vector<rs2::stream_profile> streams)
{
	for (auto &stream: streams)
	{
		std::cout << "Stream Name: " << stream.stream_name() << "; Format: " << stream.format() << "; Index: " << stream.stream_index() << "FPS: " << stream.fps() <<std::endl;
	}
}

// This example demonstrates live streaming motion data as well as video data
int live_counter()
{
	// Before running the example, check that a device supporting IMU is connected
	if (!check_imu_is_supported())
	{
		std::cerr << "Device supporting IMU (D435i) not found";
		return EXIT_FAILURE;
	}

	size_t gryro_iter = 0;
	size_t accel_iter = 0;
	size_t depth_iter = 0;
	size_t color_iter = 0;
	double ts_gyro = 0.0;
	double ts_accel = 0.0;
	double ts_depth = 0.0;
	double ts_color = 0.0;
	rs2_timestamp_domain gyro_domain;
	rs2_timestamp_domain accel_domain;
	rs2_timestamp_domain depth_domain;
	rs2_timestamp_domain color_domain;
	
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config config;
	// Add streams of gyro and accelerometer to configuration
	config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
	config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	// Add dpeth and color streams
	config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

	auto profile = config.resolve(pipe); // allows us to get device
    auto device = profile.get_device();

	auto streams =  profile.get_streams(); // 0=Depth, 1=RGB, 2=Gryo, 3=Accel
	std::cout << "Profiles that will be activated: "<< std::endl; 
	print_profiles(streams);

	// Create a mapping between sensor name and the desired profiles
	std::map<std::string, std::vector<rs2::stream_profile>> sensor_to_streams; 
	sensor_to_streams.insert(std::pair<std::string, std::vector<rs2::stream_profile>>(std::string("Stereo Module"), {streams[0]}));
	sensor_to_streams.insert(std::pair<std::string, std::vector<rs2::stream_profile>>(std::string("RGB Camera"), {streams[1]}));
	sensor_to_streams.insert(std::pair<std::string, std::vector<rs2::stream_profile>>(std::string("Motion Module"), {streams[2], streams[3]}));

	auto sensors = device.query_sensors();
	for (auto &sensor : sensors)
	{
		auto sensor_name = std::string(sensor.get_info(RS2_CAMERA_INFO_NAME));
		std::cout << "Sensor Name: " << sensor_name << std::endl;
		bool make_callback = false;
		try
		{
			// get sensor streams that are mapped to this sensor name
			auto sensor_streams = sensor_to_streams.at(sensor_name);
			std::cout << "Opening stream for " << sensor_name << std::endl;
			sensor.open(sensor_streams);
			make_callback = true;

		}
		catch(const std::exception& e)
		{
			std::cout << "Sensor " << sensor_name << " has not configured streams, skipping..." << std::endl; 
		}
		if (make_callback)
		{
			std::cout << "Creating callback for " << sensor_name << std::endl;
			sensor.start([&](rs2::frame frame){
				if (frame.get_profile().stream_type() == RS2_STREAM_COLOR)
				{
					color_iter +=1;
					ts_color = frame.get_timestamp();
					color_domain = frame.get_frame_timestamp_domain();
				}
				else if(frame.get_profile().stream_type() == RS2_STREAM_GYRO)
				{
					gryro_iter +=1;
					ts_gyro = frame.get_timestamp();
					gyro_domain = frame.get_frame_timestamp_domain();
				}
				else if(frame.get_profile().stream_type() == RS2_STREAM_ACCEL)
				{
					accel_iter +=1;
					ts_accel = frame.get_timestamp();
					accel_domain = frame.get_frame_timestamp_domain();
				}
				else if(frame.get_profile().stream_type() == RS2_STREAM_DEPTH)
				{
					depth_iter +=1;
					ts_depth = frame.get_timestamp();
					depth_domain = frame.get_frame_timestamp_domain();
				}
			});

		}
	}

	while (true)
	{
		double my_clock = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
		std::cout << std::setprecision(0) << std::fixed << "FPS --- "
			<< "Gryo: " << gryro_iter << "; Accel: " << accel_iter

			<< "; Depth: " << depth_iter << "; RGB: " << color_iter << std::endl;

		std::cout << std::setprecision(0) << std::fixed << "Timing --- Now: " << my_clock << "; Gryo: " << ts_gyro << "; Accel: " << ts_accel
			<< "; Depth: " << ts_depth << "; RGB: " << ts_color << std::endl;

		std::cout << std::setprecision(0) << std::fixed << "Time Domain --- Now: " << my_clock << "; Gryo: " << gyro_domain << "; Accel: " << accel_domain
				<< "; Depth: " << depth_domain << "; RGB: " << color_domain << std::endl;

		std::cout << std::setprecision(0) << std::fixed << "Latency --- GyroToColor: " << ts_gyro - ts_color << std::endl;
		std::cout <<std::endl;

		gryro_iter = 0;
		accel_iter = 0;
		depth_iter = 0;
		color_iter = 0;

		std::this_thread::sleep_for( 1000ms );
	}

	return EXIT_SUCCESS;
}

void OnIMUMessage(const char* topic_name_, const rstracker_pb::IMUMessage& imu_msg, const long long time_, const long long clock_)
{
	std::cout<< std::setprecision(0) << std::fixed << "Received IMUMessage; ts: " << imu_msg.ts() << std::endl;
}


}

int main(int argc, char* argv[]) try
{
	  // initialize eCAL API
	eCAL::Initialize(0, nullptr, "RSTrackerSub");
	eCAL::protobuf::CSubscriber<rstracker_pb::IMUMessage> sub_imu("IMUMessage");
  	auto imu_rec_callback = std::bind(rstracker::OnIMUMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_imu.AddReceiveCallback(imu_rec_callback);

	// create subscriber
	std::cout << "To here" << std::endl;
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	std::thread t1;
	if (FLAGS_bag == "")
	{
		rstracker::live_counter();
	}
	else
	{
		t1 = std::thread(rstracker::read_bag, FLAGS_bag);
		// t1 = std::thread(test);
		std::cout << "To here" << std::endl;
	}

	  // add receive callback function (_1 = topic_name, _2 = msg, _3 = time, , _4 = clock)
	eCAL::Process::SleepMS(1000);
	std::cout << "Before main loop" << std::endl;
    // while(true)
    // {
    //     std::this_thread::sleep_for( 1000ms );
	// 	std::cout<< "Test" <<std::endl;
    // }
	while(eCAL::Ok())
	{
		// sleep 100 ms
		eCAL::Process::SleepMS(10);
		
	}
	
	
}
catch (const rs2::error & e)

{

	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;

	return EXIT_FAILURE;
}

catch (const std::exception & e)

{

	std::cerr << e.what() << std::endl;

	return EXIT_FAILURE;
}


