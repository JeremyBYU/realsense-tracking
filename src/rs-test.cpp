#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <chrono>
#include <thread>
#include "utility.hpp"

void rs_callback(rs2::frame frame)
{
	if (frame.get_profile().stream_type() == RS2_STREAM_INFRARED)
	{
	}
	if (frame.get_profile().stream_type() == RS2_STREAM_COLOR)
	{
	}
	else if (frame.get_profile().stream_type() == RS2_STREAM_GYRO)
	{
	}
	else if (frame.get_profile().stream_type() == RS2_STREAM_ACCEL)
	{
	}
	else if (frame.get_profile().stream_type() == RS2_STREAM_DEPTH)
	{
		std::cout << std::setprecision(0) << std::fixed << "DEPTH: " << frame.get_timestamp() << std::endl;
	}
	else if (frame.get_profile().stream_type() == RS2_STREAM_POSE)
	{
		std::cout << std::setprecision(0) << std::fixed << "POSE: " << frame.get_timestamp() << std::endl;
	}
}

// D435i SN - 844212071822
// T265 SN - 943222110884

int main(int argc, char *argv[])
{
	rs2::context ctx; // Create librealsense context for managing devices
	rs2::device dev;

	// what we want to stream
	// std::vector<rstracker::StreamDetail> stream_details = {{"Intel RealSense D435I", "Depth", 640, 480, 30, RS2_FORMAT_Z16}};
	std::vector<rstracker::StreamDetail> stream_details = {{"Intel RealSense T265", "Pose", 0, 0, 200, RS2_FORMAT_6DOF}, {"Intel RealSense D435I", "Depth", 640, 480, 30, RS2_FORMAT_Z16}};
	std::vector<std::string> wanted_devices;
	std::map<std::string, std::vector<rs2::sensor>> device_sensors;
	std::transform(stream_details.begin(), stream_details.end(), std::back_inserter(wanted_devices),[](auto const& sd) { return sd.device_name; });


	std::cout << "Requesting to start: " << std::endl;
	rstracker::print_profiles(stream_details);
	std::cout << "Devices available: " << std::endl;
	auto devices = ctx.query_devices();
	for (auto dev_ : devices)
	{
		auto name = std::string(dev_.get_info(RS2_CAMERA_INFO_NAME));
		std::cout << name << ": " << dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
	}
	// Start Streaming
	for (auto dev : devices)
	{
		auto name = std::string(dev.get_info(RS2_CAMERA_INFO_NAME));
		if (std::find(wanted_devices.begin(), wanted_devices.end(), name) == wanted_devices.end())
			continue;
		// The sensors **have** to live outside of the device for loop! Therfore put inside of the std::map device_sensors
		device_sensors[name] = dev.query_sensors();
		std::cout << "Attempting to open " << name << ": " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
		for (auto &sensor : device_sensors[name])
		{
			auto sensor_name = std::string(sensor.get_info(RS2_CAMERA_INFO_NAME));
			std::cout << "Sensor Name: " << sensor_name << std::endl;
			try
			{
				auto sensor_streams = sensor.get_stream_profiles();
				rstracker::print_profiles(sensor_streams);
				std::vector<rs2::stream_profile> allowed_streams;
				for (auto &sp : sensor_streams)
				{
					auto sd_ = rstracker::stream_profile_to_details(sp);
					if (std::find(stream_details.begin(), stream_details.end(), sd_) != stream_details.end())
					{
						allowed_streams.push_back(sp);
					}
				}
				// get sensor streams that are mapped to this sensor name
				// auto sensor_streams = sensor_to_streams.at(sensor_name);
				if (allowed_streams.size() > 0)
				{
					// Open Sensor for streaming
					std::cout << "Opening stream for " << sensor_name << std::endl;
					sensor.open(allowed_streams);
					// Sensor Callback
					std::cout << "Creating callback for " << sensor_name << " streams" << std::endl;
					sensor.start([&](rs2::frame frame) { rs_callback(frame); });
				}
			}
			catch (const std::exception &e)
			{
				std::cout << "Sensor " << sensor_name << " has not configured streams, skipping..." << e.what() << std::endl;
			}
		}

	}

	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	return EXIT_SUCCESS;
}
