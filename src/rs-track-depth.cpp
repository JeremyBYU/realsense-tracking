// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
// Realsense Tracking Module

// This technique works with RSUSB driver (LIBUVC) but should work with kernel drivers as well
// This example is meant for D435i, to get high frequency motion data at the same time of depth,rgb images
// Two examples are shown, one that read a bags file and another that does live streaming from the sensor. Activate bag reading with cmd line flag --bag=<file>
// The main technique that makes this possible is to NOT USE rs::pipeline, but to directly acess each sensor and
// configure the stream manually (as well as associated callbacks).

#include <chrono>
#include <thread>
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <vector>

#include <librealsense2/rs.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
// #include <opencv2/opencv.hpp>

#include "utility.hpp"
#include "PoseMessage.pb.h"
// #include "IMUMessage.pb.h"

using namespace std::chrono_literals;
using namespace std::string_literals;

// Command line flags
DEFINE_string(bag, "", "Path to bag file");

namespace rstracker
{

void fill_pose_message(rs2_pose &pose, rstracker_pb::PoseMessage &pm, double ts)
{
	auto tr = pm.mutable_translation();
	tr->set_x(pose.translation.x);
	tr->set_y(pose.translation.y);
	tr->set_z(pose.translation.z);

	auto vl = pm.mutable_velocity();
	vl->set_x(pose.velocity.x);
	vl->set_y(pose.velocity.y);
	vl->set_z(pose.velocity.z);

	auto ac = pm.mutable_acceleration();
	ac->set_x(pose.acceleration.x);
	ac->set_y(pose.acceleration.y);
	ac->set_z(pose.acceleration.z);

	auto rot = pm.mutable_rotation();
	rot->set_x(pose.rotation.x);
	rot->set_y(pose.rotation.y);
	rot->set_z(pose.rotation.z);
	rot->set_w(pose.rotation.w);

	auto avl = pm.mutable_angular_velocity();
	avl->set_x(pose.angular_velocity.x);
	avl->set_y(pose.angular_velocity.y);
	avl->set_z(pose.angular_velocity.z);

	auto aac = pm.mutable_angular_acceleration();
	aac->set_x(pose.angular_acceleration.x);
	aac->set_y(pose.angular_acceleration.y);
	aac->set_z(pose.angular_acceleration.z);

	pm.set_tracker_confidence(pose.tracker_confidence);
	pm.set_mapper_confidence(pose.mapper_confidence);
	pm.set_hardware_ts(ts);
}

void rs_callback(rs2::frame &frame, eCAL::protobuf::CPublisher<rstracker_pb::PoseMessage> *pose_pub)
{
	if (frame.get_profile().stream_type() == RS2_STREAM_GYRO)
	{
	}
	else if (frame.get_profile().stream_type() == RS2_STREAM_ACCEL)
	{
	}
	else if (frame.get_profile().stream_type() == RS2_STREAM_POSE)
	{
		auto pframe = frame.as<rs2::pose_frame>();
		rs2_pose pose = pframe.get_pose_data();
		// std::cout << std::setprecision(0) << std::fixed << std::left << std::setw(11) << "POSE: " << frame.get_timestamp() << std::endl;

		rstracker_pb::PoseMessage pose_message;
		fill_pose_message(pose, pose_message, frame.get_timestamp());
		pose_pub->Send(pose_message);
	}
}

// int read_bag(std::string file_name)
// {
// 	// create a publisher (topic name "RSPub")
// 	LOG(INFO) << "Creating BagReader of RealSense" << std::endl;
// 	eCAL::Initialize(0, nullptr, "RSPub");
// 	eCAL::protobuf::CPublisher<rstracker_pb::PoseMessage> pub_pose("PoseMessage");
// 	// eCAL::protobuf::CPublisher<rstracker_pb::IMUMessage> pub_imu("IMUMessage");

// 	std::vector<rstracker::StreamDetail> desired_manual_streams = {{"Intel RealSense T265", "Pose", rstracker::STRM_ENUM.at("Pose"s), 0, 0, 200, RS2_FORMAT_6DOF}};
// 	rs2::context ctx;
// 	rs2::config config;
// 	rs2::device device;
// 	rs2::pipeline pipe(ctx);

// 	// enable file playback with playback repeat disabled
// 	config.enable_device_from_file(file_name, false);

// 	auto profile = config.resolve(pipe);
// 	device = profile.get_device();
// 	auto playback = device.as<rs2::playback>();
// 	playback.set_real_time(false);

// 	auto streams = profile.get_streams();
// 	print_profiles(streams);
// 	// Which sensor has the lower frame rate.
// 	auto sensors = playback.query_sensors();

// 	for (auto &sensor : sensors)
// 	{
// 		auto sensor_name = std::string(sensor.get_info(RS2_CAMERA_INFO_NAME));
// 		LOG(INFO) << "Sensor Name: " << sensor_name << std::endl;

// 		auto sensor_streams = sensor.get_stream_profiles();
// 		// rstracker::print_profiles(sensor_streams);
// 		std::vector<rs2::stream_profile> allowed_streams;
// 		// get desired stream profiles objects
// 		for (auto &sp : sensor_streams)
// 		{
// 			auto sd_ = rstracker::stream_profile_to_details(sp);
// 			if (std::find(desired_streams.begin(), desired_streams.end(), sd_) != desired_streams.end())
// 			{
// 				allowed_streams.push_back(sp);
// 			}
// 		}

// 		if (allowed_streams.size() > 0)
// 		{
// 			// Open Sensor for streaming
// 			LOG(INFO) << "Opening stream for " << sensor_name << std::endl;
// 			sensor.open(allowed_streams);
// 			// Sensor Callback
// 			LOG(INFO) << "Creating callback for " << sensor_name << " streams" << std::endl;
// 			// sensor.start([&](rs2::frame frame) { LOG(INFO) << "Inside sensor lambda"; callback(&frame);LOG(INFO) << "After callback";  });
// 			sensor.start([&pub_pose](rs2::frame frame) { rs_callback(frame, &pub_pose);});
// 		}
// 	}
// 	// This while loop is just to keep the thread alive while sensor callback are being performed
// 	while (true)
// 	{
// 		std::this_thread::sleep_for(1000ms);
// 	}
// 	return EXIT_SUCCESS;
// }

void enable_device_stream(rs2::device &dev, std::vector<std::string> &desired_devices, rs2::context &ctx,
						  std::vector<rstracker::StreamDetail> &desired_streams, std::map<std::string, std::vector<rs2::sensor>> &device_sensors,
						  std::function<void(rs2::frame)> callback)
{
	auto name = std::string(dev.get_info(RS2_CAMERA_INFO_NAME));
	if (std::find(desired_devices.begin(), desired_devices.end(), name) == desired_devices.end())
		return;
	// The sensors **have** to live outside of the device for loop! Therfore put inside of the std::map device_sensors
	device_sensors[name] = dev.query_sensors();
	LOG(INFO) << "Attempting to open " << name << ": " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
	for (auto &sensor : device_sensors[name])
	{
		auto sensor_name = std::string(sensor.get_info(RS2_CAMERA_INFO_NAME));
		LOG(INFO) << "Sensor Name: " << sensor_name << std::endl;
		try
		{
			auto sensor_streams = sensor.get_stream_profiles();
			// rstracker::print_profiles(sensor_streams);
			std::vector<rs2::stream_profile> allowed_streams;
			// get desired stream profiles objects
			for (auto &sp : sensor_streams)
			{
				auto sd_ = rstracker::stream_profile_to_details(sp);
				if (std::find(desired_streams.begin(), desired_streams.end(), sd_) != desired_streams.end())
				{
					allowed_streams.push_back(sp);
				}
			}

			if (allowed_streams.size() > 0)
			{
				// Open Sensor for streaming
				LOG(INFO) << "Opening stream for " << sensor_name << std::endl;
				sensor.open(allowed_streams);
				// Sensor Callback
				LOG(INFO) << "Creating callback for " << sensor_name << " streams" << std::endl;
				// sensor.start([&](rs2::frame frame) { LOG(INFO) << "Inside sensor lambda"; callback(&frame);LOG(INFO) << "After callback";  });
				sensor.start(callback);
			}
		}
		catch (const std::exception &e)
		{
			LOG(INFO) << "Sensor " << sensor_name << " has not configured streams, skipping..." << e.what() << std::endl;
		}
	}
}

void enable_manual_streams(rs2::context &ctx, std::vector<rstracker::StreamDetail> &desired_streams, std::map<std::string, std::vector<rs2::sensor>> &device_sensors,
						   std::function<void(rs2::frame)> callback, rs2::device *single_device = nullptr)
{
	std::vector<std::string> desired_devices;
	std::transform(desired_streams.begin(), desired_streams.end(), std::back_inserter(desired_devices), [](auto const &sd) { return sd.device_name; });

	LOG(INFO) << "Requesting to start: " << std::endl;
	rstracker::print_profiles(desired_streams);

	if (single_device)
	{
		enable_device_stream(*single_device, desired_devices, ctx, desired_streams, device_sensors, callback);
	}
	else
	{
		auto devices = ctx.query_devices();
		LOG(INFO) << "Devices available: " << std::endl;
		for (auto dev_ : devices)
		{
			auto name = std::string(dev_.get_info(RS2_CAMERA_INFO_NAME));
			LOG(INFO) << "Devices is: " << name << ": " << dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		}
		for (auto dev : devices)
		{
			enable_device_stream(dev, desired_devices, ctx, desired_streams, device_sensors, callback);
		}
	}
}

void enable_pipe_streams(std::vector<rstracker::StreamDetail> &desired_pipeline_streams, rs2::config &cfg, rs2::pipeline &pipe)
{
	for (auto &pipe_stream : desired_pipeline_streams)
	{
		cfg.enable_stream(pipe_stream.stream_type, pipe_stream.width, pipe_stream.height, pipe_stream.format, pipe_stream.fps);
	}
	// Start streaming with default recommended configuration
	pipe.start(cfg);
}

int read_bag(std::string file_name)
{
	// create a publisher (topic name "RSPub")
	LOG(INFO) << "Creating BagReader of RealSense" << std::endl;
	eCAL::Initialize(0, nullptr, "RSPub");
	eCAL::protobuf::CPublisher<rstracker_pb::PoseMessage> pub_pose("PoseMessage");
	// Create librealsense context for managing devices
	rs2::context ctx;
	// Desired streams that should be manually controlled (no pipeline, i.e., sensor.open)
	std::vector<rstracker::StreamDetail> desired_manual_streams = {{"Intel RealSense T265", "Pose", rstracker::STRM_ENUM.at("Pose"s), 0, 0, 200, RS2_FORMAT_6DOF}};
	// Desired streams that should be controlled and SYNCED through a pipeline. Like Depth and Color.
	std::vector<rstracker::StreamDetail> desired_pipeline_streams = {{"Intel RealSense D435I", "Depth", rstracker::STRM_ENUM.at("Depth"s), 640, 480, 30, RS2_FORMAT_Z16},
																	 {"Intel RealSense D435I", "Color", rstracker::STRM_ENUM.at("Color"s), 640, 480, 30, RS2_FORMAT_BGR8}};

	// needed this variable to keep sensors 'alive'
	std::map<std::string, std::vector<rs2::sensor>> device_sensors;
	auto pose_rec_callback = [&pub_pose](rs2::frame frame) { rs_callback(frame, &pub_pose); std::this_thread::sleep_for( 1000us ); };

	rs2::config config;
	rs2::device device;
	rs2::pipeline pipe(ctx);

	// enable file playback with playback repeat disabled
	config.enable_device_from_file(file_name, false);

	auto profile = config.resolve(pipe);
	device = profile.get_device();
	auto playback = device.as<rs2::playback>();
	playback.set_real_time(false);

	if (desired_manual_streams.size() > 0)
	{
		enable_manual_streams(ctx, desired_manual_streams, device_sensors, pose_rec_callback, &device);
	}

	if (desired_pipeline_streams.size() > 0)
	{
		pipe.start(config);
	}

	while (true)
	{
		if (desired_pipeline_streams.size() > 0)
		{
			auto frames = pipe.wait_for_frames();
			std::this_thread::sleep_for(1000us);
			std::cout << std::setprecision(0) << std::fixed << std::left << std::setw(11) << "FrameSet: " << frames.get_timestamp() << std::endl;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
	}
}

// This example demonstrates live streaming motion data as well as video data
int live_stream()
{
	// create a publisher (topic name "RSPub")
	LOG(INFO) << "Creating LiveStream of RealSense" << std::endl;
	eCAL::Initialize(0, nullptr, "RSPub");
	eCAL::protobuf::CPublisher<rstracker_pb::PoseMessage> pub_pose("PoseMessage");
	// Create librealsense context for managing devices
	rs2::context ctx;
	// Desired streams that should be manually controlled (no pipeline, i.e., sensor.open)
	std::vector<rstracker::StreamDetail> desired_manual_streams = {{"Intel RealSense T265", "Pose", rstracker::STRM_ENUM.at("Pose"s), 0, 0, 200, RS2_FORMAT_6DOF}};
	// Desired streams that should be controlled and SYNCED through a pipeline. Like Depth and Color.
	std::vector<rstracker::StreamDetail> desired_pipeline_streams = {{"Intel RealSense D435I", "Depth", rstracker::STRM_ENUM.at("Depth"s), 640, 480, 30, RS2_FORMAT_Z16},
																	 {"Intel RealSense D435I", "Color", rstracker::STRM_ENUM.at("Color"s), 640, 480, 30, RS2_FORMAT_BGR8}};
	// std::vector<rstracker::StreamDetail> desired_pipeline_streams;

	// needed this variable to keep sensors 'alive'
	std::map<std::string, std::vector<rs2::sensor>> device_sensors;
	auto pose_rec_callback = [&pub_pose](rs2::frame frame) { rs_callback(frame, &pub_pose); };

	if (desired_manual_streams.size() > 0)
	{
		enable_manual_streams(ctx, desired_manual_streams, device_sensors, pose_rec_callback);
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
			// std::cout << std::setprecision(0) << std::fixed << std::left << std::setw(11) << "FrameSet: " << frames.get_timestamp() << std::endl;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
	}

	return EXIT_SUCCESS;
}

// using timestamp_t = std::chrono::nanoseconds::nanoseconds;
void OnPoseMessage(const char *topic_name_, const rstracker_pb::PoseMessage &pose, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	LOG(INFO) << std::setprecision(0) << std::fixed << "Received PoseMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << pose.hardware_ts() << std::endl;
}

} // namespace rstracker

int main(int argc, char *argv[]) try
{
	google::InitGoogleLogging(argv[0]);
	LOG(INFO) << "Starting RealSense Publisher";
	// initialize eCAL API
	eCAL::Initialize(0, nullptr, "RSPub");
	// create subscriber
	eCAL::protobuf::CSubscriber<rstracker_pb::PoseMessage> sub_pose("PoseMessage");
	// add receive callback function (_1 = topic_name, _2 = msg, _3 = time, , _4 = clock)
	auto pose_rec_callback = std::bind(rstracker::OnPoseMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_pose.AddReceiveCallback(pose_rec_callback);
	// enable to receive process internal publications
	eCAL::Util::EnableLoopback(true);
	// Parse command line flags
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	std::thread t1;
	if (FLAGS_bag == "")
	{
		t1 = std::thread(rstracker::live_stream);
	}
	else
	{
		t1 = std::thread(rstracker::read_bag, FLAGS_bag);
	}

	eCAL::Process::SleepMS(1000);
	while (eCAL::Ok())
	{
		eCAL::Process::SleepMS(100);
	}
}
catch (const rs2::error &e)
{

	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;

	return EXIT_FAILURE;
}

catch (const std::exception &e)
{

	std::cerr << e.what() << std::endl;

	return EXIT_FAILURE;
}
