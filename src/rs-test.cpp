#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <chrono>
#include <thread>
#include "rspub/utility.hpp"

using namespace std::string_literals;

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
		std::cout << std::setprecision(0) << std::fixed << std::left << std::setw(11) << "DEPTH: " << frame.get_timestamp() << std::endl;
	}
	else if (frame.get_profile().stream_type() == RS2_STREAM_POSE)
	{
		std::cout << std::setprecision(0) << std::fixed << std::left << std::setw(11) << "POSE: " << frame.get_timestamp() << std::endl;
	}
}

void enable_manual_streams(rs2::context &ctx, std::vector<rspub::StreamDetail> &desired_streams, std::map<std::string, std::vector<rs2::sensor>> &device_sensors)
{
	// a vector of the devices that are being requested
	std::vector<std::string> desired_devices;
	std::transform(desired_streams.begin(), desired_streams.end(), std::back_inserter(desired_devices), [](auto const &sd) { return sd.device_name; });

	std::cout << "Requesting to start: " << std::endl;
	rspub::print_profiles(desired_streams);
	std::cout << "Devices available: " << std::endl;
	auto devices = ctx.query_devices();
	for (auto dev_ : devices)
	{
		auto name = std::string(dev_.get_info(RS2_CAMERA_INFO_NAME));
		std::cout << name << ": " << dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
	}
	// start streaming desired devices,sensors,and streams
	for (auto dev : devices)
	{
		auto name = std::string(dev.get_info(RS2_CAMERA_INFO_NAME));
		if (std::find(desired_devices.begin(), desired_devices.end(), name) == desired_devices.end())
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
				// rstracker::print_profiles(sensor_streams);
				std::vector<rs2::stream_profile> allowed_streams;
				// get desired stream profiles objects
				for (auto &sp : sensor_streams)
				{
					auto sd_ = rspub::stream_profile_to_details(sp);
					if (std::find(desired_streams.begin(), desired_streams.end(), sd_) != desired_streams.end())
					{
						allowed_streams.push_back(sp);
					}
				}

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
}

void enable_pipe_streams(std::vector<rspub::StreamDetail> &desired_pipeline_streams, rs2::config &cfg, rs2::pipeline &pipe)
{
	for (auto &pipe_stream : desired_pipeline_streams)
	{
		cfg.enable_stream(pipe_stream.stream_type, pipe_stream.width, pipe_stream.height, pipe_stream.format, pipe_stream.fps);
	}
    // Start streaming with default recommended configuration
    pipe.start(cfg);
	
}

int main(int argc, char *argv[])
{
	rs2::context ctx; // Create librealsense context for managing devices

	// std::vector<rstracker::StreamDetail> desired_streams = {{"Intel RealSense T265", "Pose", 0, 0, 200, RS2_FORMAT_6DOF}, {"Intel RealSense D435I", "Depth", 640, 480, 30, RS2_FORMAT_Z16}};
	// Desired streams that should be manually controlled (no pipeline, i.e., sensor.open)
	std::vector<rspub::StreamDetail> desired_manual_streams = {{"Intel RealSense T265", "Pose", rspub::STRM_ENUM.at("Pose"s), 0, 0, 200, rspub::FMT_ENUM.at("6DOF"s)}};
	// Desired streams that should be controlled and SYNCED through a pipeline. Like Depth and Color.
	std::vector<rspub::StreamDetail> desired_pipeline_streams = {{"Intel RealSense D435I", "Depth", rspub::STRM_ENUM.at("Depth"s), 640, 480, 30, rspub::FMT_ENUM.at("Z16"s)},
																	{"Intel RealSense D435I", "Color", rspub::STRM_ENUM.at("Color"s), 640, 480, 30, rspub::FMT_ENUM.at("BGR8"s)}};
	// std::vector<rstracker::StreamDetail> desired_pipeline_streams;

	// needed this variable to keep sensors 'alive'
	std::map<std::string, std::vector<rs2::sensor>> device_sensors;
	if (desired_manual_streams.size() > 0)
	{
		enable_manual_streams(ctx, desired_manual_streams, device_sensors);
	}

	rs2::config cfg;
	rs2::pipeline pipe(ctx);
	if (desired_pipeline_streams.size() > 0)
	{
		enable_pipe_streams(desired_pipeline_streams, cfg, pipe);
	}

	while (true)
	{
		if (desired_pipeline_streams.size() > 0)
		{
			auto frames = pipe.wait_for_frames();
			std::cout << std::setprecision(0) << std::fixed << std::left << std::setw(11) << "FrameSet: " << frames.get_timestamp() << std::endl;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		
	}

	return EXIT_SUCCESS;
}
