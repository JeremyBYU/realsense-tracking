// License: Apache 2.0. See LICENSE file in root directory.
// Realsense Module
// Subscribes to Realsense Messages and Saves to Disk

#include <chrono>
#include <thread>
#include <set>
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
#include <ecal/msg/protobuf/server.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <Open3D/Visualization/Visualizer/Visualizer.h>
#include <Open3D/Integration/ScalableTSDFVolume.h>

#include "rspub/utility.hpp"
#include "Integrate.pb.h" // the google compiler autogenerated service class

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

using namespace std::chrono_literals;
using namespace std::string_literals;

// Command line flags
DEFINE_string(config, "./config/rsintegrate_default.toml", "Path to config file");

// START Global Variables
/////////////////////////

// Keep track of previous pose changes.
double min_translate_change = 0.0;
double min_rotation_change = 0.0;
rspub_pb::Vec3 previous_pose_translation;
rspub_pb::Vec4 previous_pose_rotation;
///////////////////////
// END GLOBAL Variables

const google::protobuf::EnumDescriptor *descriptor_scene = rspub_pb::SceneRequestType_descriptor();
const google::protobuf::EnumDescriptor *descriptor_data = rspub_pb::DataRequestType_descriptor();

namespace rspub
{

using TSDFVolume = open3d::integration::ScalableTSDFVolume;
struct Scene
{
	std::shared_ptr<TSDFVolume> volume;
	enum State {STOP, START};
};

// PingService implementation
class IntegrateServiceImpl : public rspub_pb::IntegrateService
{
public:
	IntegrateServiceImpl(toml::value tcf):tcf(tcf)
	{

		// create subscriber
		sub_rgbd = rspub::SubImage("RGBDMessage");
		// auto rgbd_rec_callback = std::bind(this->OnRGBDMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
		// sub_rgbd.AddReceiveCallback(rgbd_rec_callback);
		sub_rgbd.AddReceiveCallback([this](const char *topic_name_, const rspub_pb::ImageMessage &im_msg, const long long time_, const long long clock_, const long long stuff_){ 
			OnRGBDMessage(topic_name_, im_msg, time_, clock_); 
			});
	}
	void IntegrateScene(::google::protobuf::RpcController* /* controller_ */, const rspub_pb::IntegrateRequest* request_, rspub_pb::IntegrateResponse* response_, ::google::protobuf::Closure* /* done_ */)
	{
		// print request
		LOG(INFO) << "Received Intregate Scene Request " << descriptor_scene->FindValueByNumber(request_->type())->name() 
					<< ": " << request_->scene();

		response_->set_answer("PONG");
		switch (request_->type())
		{
			case rspub_pb::SceneRequestType::ADD:
				LOG(INFO) << "Got add";
				scene_names.insert(request_->scene());
				// scenes[request_->scene()] = {std::make_shared<TSDFVolume>(), Scene::STOP};
				break;

			default:
				break;
		}
	}

	void OnRGBDMessage(const char *topic_name_, const rspub_pb::ImageMessage &im_msg, const long long time_, const long long clock_)
	{
		double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
		VLOG(1) << std::setprecision(0) << std::fixed << "Received RGBDMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << im_msg.hardware_ts() << std::endl;


		auto w = im_msg.width();
		auto h = im_msg.height();
		auto image_data_str = im_msg.image_data();
		const char *image_data_ptr = image_data_str.c_str();

		auto image_data_second_str = im_msg.image_data_second();
		const char *image_data_second_ptr = image_data_second_str.c_str();

	}
	void ExtractScene(::google::protobuf::RpcController* /* controller_ */, const rspub_pb::ExtractRequest* request_, rspub_pb::IntegrateResponse* response_, ::google::protobuf::Closure* /* done_ */)
	{
		// print request
		std::cout << "Received Extract Request" << std::endl;
		// and send response
		response_->set_answer("PONG");
	}

protected:
	std::shared_ptr<open3d::visualization::Visualizer> vis;
	std::map<std::string, Scene> scenes;
	std::set<std::string> scene_names;
	rspub::SubImage sub_rgbd;
	toml::value tcf;
	int value = 1;

};




} // namespace rspub


int main(int argc, char *argv[]) try
{
	google::InitGoogleLogging(argv[0]);
	LOG(INFO) << "Starting rs-integrate-server";
	// Parse command line flags
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	const auto tcf = toml::parse(FLAGS_config);

	min_translate_change = toml::find_or<double>(tcf, "min_translate_change", min_translate_change); // meters
	min_rotation_change = toml::find_or<double>(tcf, "min_rotation_change", min_rotation_change); // degrees
	min_rotation_change = (1.0 - cos(degreesToRadians(min_rotation_change))) / 2.0; // number between 0-1
	
	// initialize eCAL API
	eCAL::Initialize(0, nullptr, "RSIntegrate");


	  // create IntegrationServer server
	std::shared_ptr<rspub::IntegrateServiceImpl> integrate_service_impl = std::make_shared<rspub::IntegrateServiceImpl>(tcf);
	eCAL::protobuf::CServiceServer<rspub_pb::IntegrateService> integrate_service(integrate_service_impl);


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
		eCAL::Process::SleepMS(10);
	}
	eCAL::Finalize();
}
catch (const std::exception &e)
{

	std::cerr << e.what() << std::endl;

	return EXIT_FAILURE;
}
