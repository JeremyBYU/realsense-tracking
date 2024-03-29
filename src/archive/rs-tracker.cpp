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
#include <time.h>

#include <librealsense2/rs.hpp>
#include <gflags/gflags.h>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "estimator_process.h"
#include "metrics.h"
#include "tracker.h"
#include "loader.h"

#include "rspub/utility.hpp"
#include "ImageMessage.pb.h"
#include "IMUMessage.pb.h"

using namespace std::chrono_literals;

// Command line flags
DEFINE_string(bag, "", "Path to bag file");

DEFINE_string(config, "", "Path to config file");

namespace rspub
{

// This global variable holds history for collected IMU data
// TODO - Add mutex locking?
static IMUHistory imu_hist(3);

// The slow_sensor has the lower fps/frequency
// This function should be immediatly called AFTER a measurement update is given for the slow_sensor
// This measurement update is stored inside global variable imu_hist
double IMUData_LinearInterpolation(const sensor_name slow_sensor, rspub_pb::IMUMessage &imu_msg)
{
	sensor_name fast_sensor(static_cast<sensor_name>(!slow_sensor));

	// Must have 2 measurements for each sensor
	if (!imu_hist.is_data_full(slow_sensor) || !imu_hist.is_data_full(fast_sensor))
		return -1;

	IMUData slow_sensor_recent = imu_hist.recent_data(slow_sensor);
	IMUData slow_sensor_old = imu_hist.old_data(slow_sensor);

	IMUData fast_sensor_recent = imu_hist.recent_data(fast_sensor);

	if (fast_sensor_recent.ts < slow_sensor_old.ts)
	{
		// this should not happen. The fast sensor should have a more recent timestamp than the old timestamp for the slow sensor
		// Our backup is to use the previous sensor
		// std::cout<< "Fast sensor has an older timestamp than the old timestamp from the slow sensor!" << std::endl;
		// std::cout << std::setprecision(0) << std::fixed << "fast_sensor_recent.ts: " << fast_sensor_recent.ts <<  "; slow_sensor_old.ts: " << slow_sensor_old.ts <<
		// 		  "; slow_sensor_recent.ts: " << slow_sensor_recent.ts << "; Diff: " << fast_sensor_recent.ts - slow_sensor_old.ts << std::endl;

		IMUData slow_sensor_really_old = imu_hist.really_old_data(slow_sensor);
		if (fast_sensor_recent.ts < slow_sensor_really_old.ts)
		{
			// this should not happen. The fast sensor has a timestamp that is older then last 3 stamples of the slow sensor.
			std::cout << "Fast sensor has an older timestamp that is older then the last 3 samples of the slow sensor" << std::endl;
			return -1;
		}
		// std::cout << "Using a much older timestamp from the slow sensor to compensate; increased latency by a max of: " << fast_sensor_recent.ts - slow_sensor_old.ts << std::endl;
		slow_sensor_recent = slow_sensor_old;
		slow_sensor_old = slow_sensor_really_old;
	}
	if (fast_sensor_recent.ts > slow_sensor_recent.ts)
	{
		// this should not happen but might. If this happens we would just need to switch
		// the fast sensor and the slow sensor, for now just log warning
		std::cout << "Fast sensor has a newer timestamp than the most recent slow sensor!" << std::endl;
		return -1;
	}
	// Perform interpolation for the slow sensor
	double united_imu_ts = fast_sensor_recent.ts;
	double factor = (united_imu_ts - slow_sensor_old.ts) / (slow_sensor_recent.ts - slow_sensor_old.ts);
	Float3 slow_interp = slow_sensor_old.data * (1 - factor) + slow_sensor_recent.data * factor;

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

		imu_msg.set_hardware_ts(united_imu_ts);
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

		imu_msg.set_hardware_ts(united_imu_ts);
	}
	// double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	// imu_msg.set_send_ts(now);

	return united_imu_ts;
}

void rs_callback(rs2::frame frame, bool sync_with_accel, eCAL::protobuf::CPublisher<rspub_pb::ImageMessage> &pub_image, eCAL::protobuf::CPublisher<rspub_pb::IMUMessage> &pub_imu)
{
	if (frame.get_profile().stream_type() == RS2_STREAM_INFRARED)
	{
		auto vframe = frame.as<rs2::video_frame>();
		const int w = vframe.get_width();
		const int h = vframe.get_height();
		const char *image_data = static_cast<const char *>(vframe.get_data());
		int nbytes = w * h * vframe.get_bytes_per_pixel();
		auto format = vframe.get_profile().format();
		// std::cout << "Infrared bpp: " << vframe.get_bytes_per_pixel() << "; Total Bytes: " <<  nbytes << std::endl;

		rspub_pb::ImageMessage frame_image;
		frame_image.set_hardware_ts(frame.get_timestamp());
		frame_image.set_width(w);
		frame_image.set_height(h);
		frame_image.set_image_data(image_data, nbytes);
		frame_image.set_bpp(vframe.get_bytes_per_pixel());
		frame_image.set_format(format);
		pub_image.Send(frame_image);
	}
	if (frame.get_profile().stream_type() == RS2_STREAM_COLOR)
	{
		auto vframe = frame.as<rs2::video_frame>();
		const int w = vframe.get_width();
		const int h = vframe.get_height();
		const char *image_data = static_cast<const char *>(vframe.get_data());
		int nbytes = w * h * vframe.get_bytes_per_pixel();
		auto format = vframe.get_profile().format();
		// std::cout << "Color bpp: " << vframe.get_bytes_per_pixel() << "; Total Bytes: " <<  nbytes << std::endl;

		rspub_pb::ImageMessage frame_image;
		frame_image.set_hardware_ts(frame.get_timestamp());
		frame_image.set_width(w);
		frame_image.set_height(h);
		frame_image.set_image_data(image_data, nbytes);
		frame_image.set_bpp(vframe.get_bytes_per_pixel());
		frame_image.set_format(format);
		// pub_image.Send(frame_image);
	}
	else if (frame.get_profile().stream_type() == RS2_STREAM_GYRO)
	{
		rs2_vector gyro_sample = frame.as<rs2::motion_frame>().get_motion_data();
		IMUData data({gyro_sample.x, gyro_sample.y, gyro_sample.z}, frame.get_timestamp());
		imu_hist.add_data(mGYRO, data);
		if (!sync_with_accel)
		{
			rspub_pb::IMUMessage msg;
			double ts_ = IMUData_LinearInterpolation(mGYRO, msg);
			if (ts_ > 0)
			{
				// no issues with linear interpolation and the msg
				// print_message(msg, imu_hist);
				pub_imu.Send(msg);
			}
		}
	}
	else if (frame.get_profile().stream_type() == RS2_STREAM_ACCEL)
	{
		rs2_vector accel_sample = frame.as<rs2::motion_frame>().get_motion_data();
		IMUData data({accel_sample.x, accel_sample.y, accel_sample.z}, frame.get_timestamp());
		imu_hist.add_data(mACCEL, data);
		if (sync_with_accel)
		{
			rspub_pb::IMUMessage msg;
			double ts_ = IMUData_LinearInterpolation(mACCEL, msg);
			if (ts_ > 0)
			{
				// no issues with linear interpolation and the msg
				// print_message(msg, imu_hist);
				pub_imu.Send(msg);
			}
		}
	}
	else if (frame.get_profile().stream_type() == RS2_STREAM_DEPTH)
	{
	}
}

bool accel_is_slower_than_gryo(std::vector<rs2::stream_profile> &streams)
{
	auto accel_stream = std::find_if(streams.begin(), streams.end(),
									 [](const rs2::stream_profile &x) { return x.stream_type() == RS2_STREAM_ACCEL; });
	auto gryo_stream = std::find_if(streams.begin(), streams.end(),
									[](const rs2::stream_profile &x) { return x.stream_type() == RS2_STREAM_GYRO; });
	if (accel_stream == streams.end() || gryo_stream == streams.end())
	{
		// This bag file doesn't have any gryo and accelerometer data! Exit early.
		std::cout << "This data stream (bag or live) does not have any gyro or accel data. Exiting";
		throw "This data stream (bag or live) does not have any gyro or accel data. Exiting";
	}

	bool sync_with_accel = accel_stream->fps() <= gryo_stream->fps();
	return sync_with_accel;
}

int read_bag(std::string file_name)
{
	// create a publisher (topic name "person")
	std::cout << "Read bag" << std::endl;
	eCAL::Initialize(0, nullptr, "RSTrackerPub");
	eCAL::protobuf::CPublisher<rspub_pb::ImageMessage> pub_image("ImageMessage");
	eCAL::protobuf::CPublisher<rspub_pb::IMUMessage> pub_imu("IMUMessage");
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
	print_profiles(streams);
	// Which sensor has the lower frame rate.
	bool sync_with_accel = accel_is_slower_than_gryo(streams);
	auto sensors = playback.query_sensors();

	std::cout << "Syncing with accel: " << sync_with_accel << std::endl;

	for (auto &sensor : sensors)
	{
		auto profiles = sensor.get_stream_profiles();
		for (auto &profile : profiles)
		{
			sensor.open(profile);
			std::cout << "Profile: " << profile.stream_name() << std::endl;
		}
		// Sensor Callback
		sensor.start([&](rs2::frame frame) {
			rs_callback(frame, sync_with_accel, pub_image, pub_imu);
			std::this_thread::sleep_for(1000us);
		});
	}
	// This while loop is just to keep the thread alive while sensor callback are being performed
	while (true)
	{
		std::this_thread::sleep_for(1000ms);
	}
	return EXIT_SUCCESS;
}

// This example demonstrates live streaming motion data as well as video data
int live_stream()
{
	// create a publisher (topic name "person")
	std::cout << "Creating LiveStream of RealSense" << std::endl;
	eCAL::Initialize(0, nullptr, "RSTrackerPub");
	eCAL::protobuf::CPublisher<rspub_pb::ImageMessage> pub_image("ImageMessage");
	eCAL::protobuf::CPublisher<rspub_pb::IMUMessage> pub_imu("IMUMessage");
	std::cout << "Make Publisher" << std::endl;
	// Before running the example, check that a device supporting IMU is connected
	if (!check_imu_is_supported())
	{
		std::cerr << "Device supporting IMU (D435i) not found";
		return EXIT_FAILURE;
	}

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config config;
	// Add streams of gyro and accelerometer to configuration
	config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);
	config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	// Add dpeth and color streams
	config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	config.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);

	auto profile = config.resolve(pipe); // allows us to get device
	auto device = profile.get_device();

	auto streams = profile.get_streams(); // 0=Depth, 1=RGB, 2=Gryo, 3=Accel
	std::cout << "Profiles that will be activated: " << std::endl;
	print_profiles(streams);
	std::cout << std::endl;

	bool sync_with_accel = accel_is_slower_than_gryo(streams);

	std::map<std::string, std::string> stream_to_sensor_name = {{"Depth", "Stereo Module"}, {"Infrared 1", "Stereo Module"}, {"Color", "RGB Camera"}, {"Gyro", "Motion Module"}, {"Accel", "Motion Module"}, {"Pose", "Tracking Module"}};
	std::map<std::string, std::vector<rs2::stream_profile>> sensor_to_streams;
	for (auto &stream_profile : streams)
	{
		auto stream_name = stream_profile.stream_name();
		auto sensor_name_str = stream_to_sensor_name[stream_name];
		try
		{
			// get sensor streams that are mapped to this sensor name
			std::vector<rs2::stream_profile> &sensor_streams = sensor_to_streams.at(sensor_name_str);
			sensor_streams.push_back(stream_profile);
		}
		catch (const std::exception &e)
		{
			sensor_to_streams.insert(std::pair<std::string, std::vector<rs2::stream_profile>>(sensor_name_str, {stream_profile}));
		}
	}

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
			print_profiles(sensor_streams);
			sensor.open(sensor_streams);
			make_callback = true;
		}
		catch (const std::exception &e)
		{
			std::cout << "Sensor " << sensor_name << " has not configured streams, skipping..." << std::endl;
		}
		if (make_callback)
		{
			std::cout << "Creating callback for " << sensor_name << std::endl;
			// Sensor Callback
			sensor.start([&](rs2::frame frame) {
				rs_callback(frame, sync_with_accel, pub_image, pub_imu);
			});
		}
	}

	// This while loop is just to keep the thread alive while sensor callback are being performed
	while (true)
	{
		std::this_thread::sleep_for(1000ms);
	}
	return EXIT_SUCCESS;
}
auto ESTIMATOR_CFG = xivo::LoadJson("/opt/workspace/config/xivo_d435i.json");
auto ESTIMATOR = xivo::CreateSystem(ESTIMATOR_CFG);
// const auto window_name = "Display Image";

template <typename T>
auto nanoseconds_to_duration(T ns)
{
	return std::chrono::duration<T, std::ratio<1, 1000000000>>(ns);
}

void OnIMUMessage(const char *topic_name_, const rspub_pb::IMUMessage &imu_msg, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	// std::cout<< std::setprecision(0) << std::fixed << "Received IMUMessage; now: " << now << "; send_ts: " << time_/1000  << "; hardware_ts: " << imu_msg.hardware_ts()  << std::endl;
	xivo::timestamp_t ts_ = xivo::timestamp_t(static_cast<uint64_t>(imu_msg.hardware_ts() * MS_TO_NS));
	// LOG(INFO) << "New ts: " << ts_.count();
	xivo::Vec3 gyro(imu_msg.gyro().x(), imu_msg.gyro().y(), imu_msg.gyro().z());
	xivo::Vec3 accel(imu_msg.accel().x(), imu_msg.accel().y(), imu_msg.accel().z());
	ESTIMATOR->InertialMeas(ts_, gyro, accel);
}

// using timestamp_t = std::chrono::nanoseconds::nanoseconds;
void OnImageMessage(const char *topic_name_, const rspub_pb::ImageMessage &img, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	auto w = img.width();
	auto h = img.height();
	auto image_data_str = img.image_data();
	const char *image_data_ptr = image_data_str.c_str();
	auto img_size_bytes = image_data_str.size();
	cv::Mat image(cv::Size(w, h), CV_8UC1, (void *)image_data_ptr, cv::Mat::AUTO_STEP);
	// std::cout<< std::setprecision(0) << std::fixed << "Received ImageMessage; now: " << now << "; send_ts: " << time_/1000  << "; hardware_ts: " << img.hardware_ts()  << std::endl;
	// std::cout<< std::setprecision(0) << std::fixed << "Image size: " << img_size_bytes << std::endl;
	// cv::imshow(window_name, image);
	// cv::waitKey(1);
	xivo::timestamp_t ts_ = xivo::timestamp_t(static_cast<uint64_t>(img.hardware_ts() * MS_TO_NS));
	// LOG(INFO) << "New ts: " << ts_.count();
	ESTIMATOR->VisualMeas(ts_, image);
	auto gsb_ = ESTIMATOR->gsb();
	auto translation = gsb_.translation();
	std::cout << "New translation: " << translation << std::endl;
}

} // namespace rspub

int main(int argc, char *argv[]) try
{
	// cv::namedWindow(rstracker::window_name, cv::WINDOW_AUTOSIZE);
	google::InitGoogleLogging(argv[0]);
	LOG(INFO) << "Starting RealSense Tracker";
	// initialize eCAL API
	eCAL::Initialize(0, nullptr, "RSTrackerSub");
	// create subscriber
	eCAL::protobuf::CSubscriber<rspub_pb::IMUMessage> sub_imu("IMUMessage");
	eCAL::protobuf::CSubscriber<rspub_pb::ImageMessage> sub_image("ImageMessage");
	// add receive callback function (_1 = topic_name, _2 = msg, _3 = time, , _4 = clock)
	auto imu_rec_callback = std::bind(rspub::OnIMUMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_imu.AddReceiveCallback(imu_rec_callback);
	auto image_rec_callback = std::bind(rspub::OnImageMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_image.AddReceiveCallback(image_rec_callback);

	// enable to receive process internal publications
	eCAL::Util::EnableLoopback(true);

	gflags::ParseCommandLineFlags(&argc, &argv, true);
	std::thread t1;
	if (FLAGS_bag == "")
	{
		t1 = std::thread(rspub::live_stream);
	}
	else
	{
		t1 = std::thread(rspub::read_bag, FLAGS_bag);
	}

	eCAL::Process::SleepMS(1000);
	while (eCAL::Ok())
	{
		// sleep 100 ms
		eCAL::Process::SleepMS(10);
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
