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
#include <math.h>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <vector>
#include <atomic>

#include <librealsense2/rs.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>

#include "rspub/utility.hpp"

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

using namespace std::chrono_literals;
using namespace std::string_literals;

// Command line flags
DEFINE_string(path, "", "Path to save images");
DEFINE_string(config, "./config/rssave_default.toml", "Path to config file");

// START Global Variables
/////////////////////////
std::map<std::string, std::string> folder_paths;
const std::string pose_fname = "trajectory";

int depth_counter = 0;
int rgb_counter = 0;
int rgbd_counter = 0;
int pose_counter = 0;
int point_counter = 0;
int pose_per_file = 1000;
int pointcloud_per_file = 1;

std::vector<rspub_pb::PoseMessage> pose_messages;
rspub_pb::PoseMessageList pose_list;

// Keep track of previous pose changes.
double min_translate_change = 0.0;
double min_rotation_change = 0.0;
rspub_pb::Vec3 previous_pose_translation;
rspub_pb::Vec4 previous_pose_rotation;
// thread safe flags to let each thread know
// that a pose has sufficiently changed.
std::atomic<bool> pose_changed_pc(false);
std::atomic<bool> pose_changed_color(false);
std::atomic<bool> pose_changed_depth(false);
std::atomic<bool> pose_changed_rgbd(false);
///////////////////////
// END GLOBAL Variables

namespace rspub
{

// quaternion inner product squared
double quat_product(const double &x1, const double &y1, const double &z1, const double &w1, const double &x2, const double &y2, const double &z2, const double &w2)
{
	double inner_product = x1*x2 + y1*y2 + z1*z2 + w1*w2;
	double inner_product_2 = inner_product * inner_product;
	return inner_product_2;
}

// Approximate radian difference between two quaternions
// https://math.stackexchange.com/questions/90081/quaternion-distance
// NOTE Must be unit quaternions
double quat_angle_diff(rspub_pb::Vec4 &quat1, rspub_pb::Vec4 &quat2)
{
	double inner_product_2 = quat_product(quat1.x(), quat1.y(), quat1.z(), quat1.w(), quat2.x(), quat2.y(), quat2.z(), quat2.w());
	return 1.0 - inner_product_2;
}

double norm_l2(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2)
{
	double dx = x2-x1;
	double dy = y2-y1;
	double dz = z2-z1;
	return std::sqrt(dx*dx + dy*dy + dz*dz);
}


std::string pad_int(int num, int pad = 8)
{
	std::ostringstream out;
	out << std::internal << std::setfill('0') << std::setw(pad) << num;
	return out.str();
}

template <class T>
int WriteProtoData(T &proto_data, std::string fpath)
{
	std::fstream output(fpath, std::ios::out | std::ios::trunc | std::ios::binary);
	if (!proto_data.SerializeToOstream(&output))
	{
		std::cerr << "Failed to write proto data data." << std::endl;
		return -1;
	}
}

void OnPoseMessage(const char *topic_name_, const rspub_pb::PoseMessage &pose_msg, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	VLOG(1)  << std::setprecision(0) << std::fixed << "Received PoseMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << pose_msg.hardware_ts() << std::endl;

	auto new_pose = pose_list.add_poses();
	*new_pose = pose_msg;

	if ((pose_counter % pose_per_file) == 0)
	{
		auto fpath = folder_paths["pose"] + pose_fname + "_" + pad_int(pose_counter / pose_per_file) + ".pb";
		WriteProtoData<rspub_pb::PoseMessageList>(pose_list, fpath);
		pose_list.clear_poses();
	}

	// Check if pose is sufficiently 'different' from previous
	auto pose_translation = pose_msg.translation();
	auto pose_rotation = pose_msg.rotation();
	auto translate_change = norm_l2(previous_pose_translation.x(), previous_pose_translation.y(), previous_pose_translation.z(), 
								pose_translation.x(), pose_translation.y(), pose_translation.z());
	auto rotation_change = quat_angle_diff(previous_pose_rotation, pose_rotation);
	VLOG(2) << "pose change; translate: " << translate_change << "; rotation_change: " << rotation_change;
	if (translate_change > min_translate_change || (rotation_change > min_rotation_change))
	{
		pose_changed_rgbd = true; // Set atomic flag
		pose_changed_depth = true; // Set atomic flag
		pose_changed_color = true; // Set atomic flag
		pose_changed_pc = true; // Set atomic flag

		previous_pose_translation = pose_translation;
		previous_pose_rotation = pose_rotation;
		LOG(INFO) << "Sufficiently different pose";
		// Have a moderately different pose
	}


	pose_counter++;
}

void write_img(const char *image_data_ptr, int w, int h, int d_type, int counter, std::string sensor_type, double ts)
{
	try
	{
		cv::Mat image(cv::Size(w, h), d_type, (void *)image_data_ptr, cv::Mat::AUTO_STEP);
		// TODO use string stream
		std::string time_stamp = std::to_string(static_cast<std::int64_t>(std::floor(ts)));
		auto fpath = folder_paths[sensor_type] + pad_int(counter) + "_" + time_stamp + ".png";
		cv::imwrite(fpath, image);
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << '\n';
	}
}

void OnDepthMessage(const char *topic_name_, const rspub_pb::ImageMessage &im_msg, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	VLOG(1)  << std::setprecision(0) << std::fixed << "Received DepthMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << im_msg.hardware_ts() << std::endl;

	if (!pose_changed_depth.load())
		return;
	pose_changed_depth = false;

	auto w = im_msg.width();
	auto h = im_msg.height();
	auto image_data_str = im_msg.image_data();
	const char *image_data_ptr = image_data_str.c_str();
	write_img(image_data_ptr, w, h, CV_16UC1, depth_counter, "depth"s, im_msg.hardware_ts());
	depth_counter++;
}

void OnColorMessage(const char *topic_name_, const rspub_pb::ImageMessage &im_msg, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	VLOG(1)  << std::setprecision(0) << std::fixed << "Received ColorMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << im_msg.hardware_ts() << std::endl;

	if (!pose_changed_color.load())
		return;
	pose_changed_color = false;

	auto w = im_msg.width();
	auto h = im_msg.height();
	auto image_data_str = im_msg.image_data();
	const char *image_data_ptr = image_data_str.c_str();
	write_img(image_data_ptr, w, h, CV_8UC3, rgb_counter, "color"s, im_msg.hardware_ts());
	rgb_counter++;
}

void OnRGBDMessage(const char *topic_name_, const rspub_pb::ImageMessage &im_msg, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	VLOG(1) << std::setprecision(0) << std::fixed << "Received RGBDMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << im_msg.hardware_ts() << std::endl;

	if (!pose_changed_rgbd.load())
		return;
	pose_changed_rgbd = false;

	auto w = im_msg.width();
	auto h = im_msg.height();
	auto image_data_str = im_msg.image_data();
	const char *image_data_ptr = image_data_str.c_str();
	write_img(image_data_ptr, w, h, CV_8UC3, rgbd_counter, "color"s, im_msg.hardware_ts());

	auto image_data_second_str = im_msg.image_data_second();
	const char *image_data_second_ptr = image_data_second_str.c_str();
	write_img(image_data_second_ptr, w, h, CV_16UC1, rgbd_counter, "depth"s, im_msg.hardware_ts());

	rgbd_counter++;
}

void OnPointCloudMessage(const char *topic_name_, const rspub_pb::PointCloudMessage &pc_msg, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	VLOG(1) << std::setprecision(0) << std::fixed << "Received PointCloudMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << pc_msg.hardware_ts() << std::endl;

	if (!pose_changed_pc.load())
		return;
	pose_changed_pc = false;

	point_counter++;

	if ((point_counter % pointcloud_per_file) == 0)
	{
		auto fpath = folder_paths["pointcloud"s] + pad_int(point_counter, 8) + ".pb";
		WriteProtoData<rspub_pb::PoseMessageList>(pose_list, fpath);
	}
}

} // namespace rspub

void create_folders(std::string &base_path, std::map<std::string, std::string> &folder_paths)
{
	cv::utils::fs::remove_all(base_path);
	cv::utils::fs::createDirectory(base_path);
	auto color_path = base_path + "color/";
	auto depth_path = base_path + "depth/";
	auto pose_path = base_path + "scene/";
	auto pointcloud_path = base_path + "pointcloud/";

	cv::utils::fs::createDirectory(color_path);
	cv::utils::fs::createDirectory(depth_path);
	cv::utils::fs::createDirectory(pose_path);
	cv::utils::fs::createDirectory(pointcloud_path);

	folder_paths["color"] = color_path;
	folder_paths["depth"] = depth_path;
	folder_paths["pose"] = pose_path;
	folder_paths["pointcloud"] = pointcloud_path;
}

int main(int argc, char *argv[]) try
{
	google::InitGoogleLogging(argv[0]);
	LOG(INFO) << "Starting rs-save";
	// Parse command line flags
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	const auto tcf = toml::parse(FLAGS_config);
	std::string base_path = toml::find_or(tcf, "base_path", "");
	min_translate_change = toml::find_or<double>(tcf, "min_translate_change", min_translate_change);
	min_rotation_change = toml::find_or<double>(tcf, "min_rotation_change", min_rotation_change); // degrees
	// LOG(INFO) << pose_rotation_change;
	min_rotation_change = (1.0 - cos(degreesToRadians(min_rotation_change))) / 2.0; // number between 0-1
	// LOG(INFO) << min_translate_change;
	// LOG(INFO) << min_rotation_change;
	// LOG(INFO) << pose_rotation_change;
	// LOG(INFO) << base_path;

	if (base_path == "")
	{
		LOG(INFO) << "No path specified, creating time stamped folder";
		std::stringstream ss;
		time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		ss << std::put_time(localtime(&now), "%F_%T/");
		base_path = FLAGS_path + ss.str();
		return 1;
	}
	else
	{
		char ch = base_path.back();
		if (ch != '/')
			base_path = base_path + '/';
	}
	create_folders(base_path, folder_paths);
	LOG(INFO) << "Using this data folder: " << base_path;

	// initialize eCAL API
	eCAL::Initialize(0, nullptr, "RSSave");
	// create subscriber
	rspub::SubPose sub_pose("PoseMessage");
	rspub::SubImage sub_depth("DepthMessage");
	rspub::SubImage sub_color("ColorMessage");
	rspub::SubPointCloud sub_pc("PointCloudMessage");
	rspub::SubImage sub_rgbd("RGBDMessage");
	// add receive callback function (_1 = topic_name, _2 = msg, _3 = time, , _4 = clock)
	auto pose_rec_callback = std::bind(rspub::OnPoseMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_pose.AddReceiveCallback(pose_rec_callback);
	auto pc_rec_callback = std::bind(rspub::OnPointCloudMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_pc.AddReceiveCallback(pc_rec_callback);
	auto depth_rec_callback = std::bind(rspub::OnDepthMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_depth.AddReceiveCallback(depth_rec_callback);
	auto color_rec_callback = std::bind(rspub::OnColorMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_color.AddReceiveCallback(color_rec_callback);
	auto rgbd_rec_callback = std::bind(rspub::OnRGBDMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_rgbd.AddReceiveCallback(rgbd_rec_callback);
	// enable to receive process internal publications


	// Have to set the global variable here this way
	previous_pose_translation.set_x(0);
	previous_pose_translation.set_y(0);
	previous_pose_translation.set_z(0);

	previous_pose_rotation.set_x(0);
	previous_pose_rotation.set_y(0);
	previous_pose_rotation.set_z(0);
	previous_pose_rotation.set_w(0);


	eCAL::Process::SleepMS(1000);
	while (eCAL::Ok())
	{
		eCAL::Process::SleepMS(100);
	}
}
catch (const std::exception &e)
{

	std::cerr << e.what() << std::endl;

	return EXIT_FAILURE;
}
