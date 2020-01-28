// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
// After many many times of trial and error I have finally arrived on this technique to get multiple sensor streams from
// an intel realsense which have DIFFERENT FRAME RATES

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
#include <iostream>
#include <iomanip>

#include <librealsense2/rs.hpp>
#include <gflags/gflags.h>

using namespace std::chrono_literals;

// Command line flags
DEFINE_string(bag, "",
				"Path to bag file");

bool check_imu_is_supported()
{

	bool found_gyro = false;
	bool found_accel = false;
	rs2::context ctx;
	for (auto dev : ctx.query_devices())
	{
		// The same device should support gyro and accel
		found_gyro = false;
		found_accel = false;
		for (auto sensor : dev.query_sensors())
		{
			for (auto profile : sensor.get_stream_profiles())
			{
				if (profile.stream_type() == RS2_STREAM_GYRO)
					found_gyro = true;
				if (profile.stream_type() == RS2_STREAM_ACCEL)
					found_accel = true;
			}
		}
		if (found_gyro && found_accel)
			break;
	}
	return found_gyro && found_accel;
}


int bag_counter(std::string file_name)
{

	size_t gyro_iter = 1;
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

	rs2::config config;
	rs2::device device;
	rs2::pipeline pipe;

	// enable file playback with playback repeat disabled
	config.enable_device_from_file(file_name, false);

	auto profile = config.resolve(pipe);
    device = profile.get_device();
    auto playback = device.as<rs2::playback>();
    playback.set_real_time(false);
	// DONT START THE PIPELINE, you will always get dropped frames from high frequency data if paired with low frequency images
	// pipeline_profile = pipe.start( config );

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
				color_iter +=1;
				ts_color = frame.get_timestamp();
				color_domain = frame.get_frame_timestamp_domain();
			}
			else if(frame.get_profile().stream_type() == RS2_STREAM_GYRO)
			{
				gyro_iter +=1;
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
			std::this_thread::sleep_for( 1ms );

		});
	}

	while (true)
	{
		
		double my_clock = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
		std::cout << std::setprecision(0) << std::fixed << "FPS --- "
			<< "Gyro: " << gyro_iter << "; Accel: " << accel_iter

			<< "; Depth: " << depth_iter << "; RGB: " << color_iter << std::endl;

		std::cout << std::setprecision(0) << std::fixed << "Timing --- Now: " << my_clock << "; Gyro: " << ts_gyro << "; Accel: " << ts_accel
			<< "; Depth: " << ts_depth << "; RGB: " << ts_color << std::endl;

		std::cout << std::setprecision(0) << std::fixed << "Time Domain --- Now: " << my_clock << "; Gyro: " << gyro_domain << "; Accel: " << accel_domain
				<< "; Depth: " << depth_domain << "; RGB: " << color_domain << std::endl;

		std::cout << std::setprecision(0) << std::fixed << "Latency --- GyroToColor: " << ts_gyro - ts_color << std::endl;
		std::cout <<std::endl;
		if (gyro_iter == 0)
			break;
		gyro_iter = 0;
		accel_iter = 0;
		depth_iter = 0;
		color_iter = 0;

		std::this_thread::sleep_for( 1000ms );
	}

	return EXIT_SUCCESS;
}


void print_profiles(std::vector<rs2::stream_profile> streams)
{
	for (auto &stream: streams)
	{
		std::cout << "Stream Name: " << stream.stream_name() << "; Format: " << stream.format() << "; Index: " << stream.stream_index() << "; FPS: " << stream.fps() <<std::endl;
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

	size_t gyro_iter = 0;
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

	auto streams =  profile.get_streams(); // 0=Depth, 1=RGB, 2=Gyro, 3=Accel
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
			double old_clock = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
			sensor.start([&](rs2::frame frame){
				double my_clock = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
				if (frame.get_profile().stream_type() == RS2_STREAM_COLOR)
				{
					color_iter +=1;
					ts_color = frame.get_timestamp();
					color_domain = frame.get_frame_timestamp_domain();
				}
				else if(frame.get_profile().stream_type() == RS2_STREAM_GYRO)
				{
					gyro_iter +=1;
					ts_gyro = frame.get_timestamp();
					gyro_domain = frame.get_frame_timestamp_domain();
					if ((my_clock - old_clock) > 1000)
					{
						std::cout << std::setprecision(0) << std::fixed << "now:" << my_clock
						<< "; Gyro ts: " << ts_gyro << std::endl;
						old_clock = my_clock;

					}
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
			<< "Gyro: " << gyro_iter << "; Accel: " << accel_iter

			<< "; Depth: " << depth_iter << "; RGB: " << color_iter << std::endl;

		std::cout << std::setprecision(0) << std::fixed << "Timing --- Now: " << my_clock << "; Gyro: " << ts_gyro << "; Accel: " << ts_accel
			<< "; Depth: " << ts_depth << "; RGB: " << ts_color << std::endl;

		std::cout << std::setprecision(0) << std::fixed << "Time Domain --- Now: " << my_clock << "; Gyro: " << gyro_domain << "; Accel: " << accel_domain
				<< "; Depth: " << depth_domain << "; RGB: " << color_domain << std::endl;

		std::cout << std::setprecision(0) << std::fixed << "Latency --- GyroToColor: " << ts_gyro - ts_color << std::endl;
		std::cout <<std::endl;

		gyro_iter = 0;
		accel_iter = 0;
		depth_iter = 0;
		color_iter = 0;

		std::this_thread::sleep_for( 1000ms );
	}

	return EXIT_SUCCESS;
}

int main(int argc, char* argv[]) try
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	if (FLAGS_bag == "")
	{
		live_counter();
	}
	else
	{
		bag_counter(FLAGS_bag);
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
