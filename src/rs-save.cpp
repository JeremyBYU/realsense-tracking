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
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>

#include "rspub/utility.hpp"


using namespace std::chrono_literals;
using namespace std::string_literals;

// Command line flags
DEFINE_string(path, "", "Path to save images");

// global variable
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


namespace rspub
{


std::string pad_int(int num, int pad=8) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(pad) << num;
  return out.str();
}

// int WritePoseData(rspub_pb::PoseMessageList &pose_list, std::string base_folder, int pose_counter)
// {
// 	auto fpath = base_folder + pose_fname + "_" + pad_int(pose_counter / pose_per_file ) +".log";
// 	std::cout << fpath << std::endl;
//     std::fstream output(fpath, std::ios::out | std::ios::trunc | std::ios::binary);
//     if (!pose_list.SerializeToOstream(&output)) {
//       std::cerr << "Failed to write pose data." << std::endl;
//       return -1;
//     }
// }

template<class T>
int WriteProtoData(T &proto_data, std::string fpath)
{
    std::fstream output(fpath, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!proto_data.SerializeToOstream(&output)) {
      std::cerr << "Failed to write proto data data." << std::endl;
      return -1;
    }
}

// using timestamp_t = std::chrono::nanoseconds::nanoseconds;
void OnPoseMessage(const char *topic_name_, const rspub_pb::PoseMessage &pose_msg, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	LOG(INFO) << std::setprecision(0) << std::fixed << "Received PoseMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << pose_msg.hardware_ts() << std::endl;

	auto new_pose = pose_list.add_poses();
	*new_pose = pose_msg;

	pose_counter++;

	if ((pose_counter % pose_per_file) == 0)
	{
		auto fpath = folder_paths["pose"] + pose_fname + "_" + pad_int(pose_counter / pose_per_file ) +".pb";
		WriteProtoData<rspub_pb::PoseMessageList>(pose_list, fpath);
		pose_list.clear_poses();
	}
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
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	
}

void OnDepthMessage(const char *topic_name_, const rspub_pb::ImageMessage &im_msg, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	LOG(INFO) << std::setprecision(0) << std::fixed << "Received DepthMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << im_msg.hardware_ts() << std::endl;

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
	LOG(INFO) << std::setprecision(0) << std::fixed << "Received ColorMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << im_msg.hardware_ts() << std::endl;

	auto w = im_msg.width();
	auto h = im_msg.height();
	auto image_data_str = im_msg.image_data();
	const char *image_data_ptr = image_data_str.c_str();
	write_img(image_data_ptr, w, h, CV_8UC3, rgb_counter, "color"s, im_msg.hardware_ts());
	rgb_counter++;

}



void OnPointCloudMessage(const char *topic_name_, const rspub_pb::PointCloudMessage &pc_msg, const long long time_, const long long clock_)
{
	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	LOG(INFO) << std::setprecision(0) << std::fixed << "Received PointCloudMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << pc_msg.hardware_ts() << std::endl;
	
	point_counter++;

	if ((point_counter % pointcloud_per_file) == 0)
	{
		auto fpath = folder_paths["pointcloud"s] + pad_int(point_counter, 8) +".pb";
		WriteProtoData<rspub_pb::PoseMessageList>(pose_list, fpath);
	}
}


} // namespace rstracker

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
	std::string base_path;
	if(FLAGS_path == "")
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
		base_path = FLAGS_path;
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
	rspub::SubPointCloud sub_pc("PointCloudMessage");
	rspub::SubImage sub_depth("DepthMessage");
	rspub::SubImage sub_color("ColorMessage");
	// add receive callback function (_1 = topic_name, _2 = msg, _3 = time, , _4 = clock)
	auto pose_rec_callback = std::bind(rspub::OnPoseMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_pose.AddReceiveCallback(pose_rec_callback);
	auto pc_rec_callback = std::bind(rspub::OnPointCloudMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_pc.AddReceiveCallback(pc_rec_callback);
	auto depth_rec_callback = std::bind(rspub::OnDepthMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_depth.AddReceiveCallback(depth_rec_callback);
	auto color_rec_callback = std::bind(rspub::OnColorMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
	sub_color.AddReceiveCallback(color_rec_callback);
	// enable to receive process internal publications
	
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
