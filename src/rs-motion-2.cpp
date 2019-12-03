// License: Apache 2.0. See LICENSE file in root directory.

// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <cstring>
#include <chrono>
#include <thread>
#include <mutex>

#include <librealsense2/rs.hpp>
#include <gflags/gflags.h>



#include "example.hpp" // Include short list of convenience functions for rendering

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

	size_t gryro_iter = 1;
	size_t accel_iter = 0;
	size_t depth_iter = 0;
	size_t color_iter = 0;
	double ts_gyro = 0.0;
	double ts_accel = 0.0;
	double ts_depth = 0.0;
	double ts_color = 0.0;
	double depth_gryo_latency = 0.0;
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
	// start pipeline and get device
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
			std::this_thread::sleep_for( 1ms );

		});
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
		if (gryro_iter == 0)
			break;
		gryro_iter = 0;
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
		std::cout << "Stream Name: " << stream.stream_name() << "; Format: " << stream.format() << "; Index: " << stream.stream_index() << "FPS: " << stream.fps() <<std::endl;
	}
}

int live_counter()
{
	using namespace std::chrono_literals;

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
	double depth_gryo_latency = 0.0;
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
	// Declare object that handles camera pose calculations
	config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

	auto profile = config.resolve(pipe);
    auto device = profile.get_device();

	auto streams =  profile.get_streams(); // 0=Depth, 1=RGB, 2=Gryo, 3=Accel
	std::cout << "Profiles that will be activated: "<< std::endl; 
	print_profiles(streams);

	auto sensors = device.query_sensors();
	for (auto &sensor : sensors)
	{
		auto sensor_name = std::string(sensor.get_info(RS2_CAMERA_INFO_NAME));
		std::cout << "Sensor Name: " << sensor_name << std::endl;
		bool make_callback = false;
		if (sensor_name == std::string("Stereo Module")) // This will be first
		{
			std::cout << "Opening stream for " << sensor_name << std::endl;
			sensor.open(streams[0]);
			streams.erase(streams.begin());
			make_callback = true;
		}
		if (sensor_name == std::string("RGB Camera")) // This will be second
		{
			std::cout << "Opening stream for " << sensor_name << std::endl;
			sensor.open(streams[0]);
			streams.erase(streams.begin());
			make_callback = true;
		}
		if (sensor_name == std::string("Motion Module")) // This will be last
		{
			std::cout << "Opening stream for " << sensor_name << std::endl;
			sensor.open(streams);
			make_callback = true;
		}
		if (make_callback)
		{
			std::cout << "Creating callback for " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
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
