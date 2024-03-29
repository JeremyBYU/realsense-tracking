// License: MIT. See LICENSE file in root directory.
// Integrate RGBD Frames into Volume and Mesh

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
#include <open3d/visualization/visualizer/Visualizer.h>
#include <open3d/pipelines/integration/ScalableTSDFVolume.h>
#include <open3d/io/IJsonConvertibleIO.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/io/ImageIO.h>
#include <open3d/geometry/HalfEdgeTriangleMesh.h>
#include <Eigen/Geometry>

#include "rspub/utility.hpp"
#include "Integrate.pb.h" // the google compiler autogenerated service class
// #include "polylidar/polylidar.hpp"
// #include "polylidar/util.hpp"

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

using namespace std::chrono_literals;
using namespace std::string_literals;
namespace o3d = open3d;
namespace o3dGeom = o3d::geometry;

// Command line flags
DEFINE_string(config, "./config/rsintegrate_default.toml", "Path to config file");
DEFINE_bool(force_udp, false, "Force UDP Multicast for publishers");

// START Global Variables
/////////////////////////

static Eigen::Matrix4d H_t265_d400 = (Eigen::Matrix4d() << 1.0, 0, 0, 0,
									  0, -1.0, 0, 0,
									  0, 0, -1.0, 0,
									  0, 0, 0, 1)
										 .finished();

static Eigen::Matrix4d pre_multiply = (Eigen::Matrix4d() << 
									  1.0, 0, 0, 0,
									  0, 1.0, 0, 0,
									  0, 0, 1.0, 0,
									  0, 0, 0, 1.0)
										 .finished();

static Eigen::Matrix4d post_multiply = (Eigen::Matrix4d() << 
									  1.0, 0, 0, 0,
									  0, 1.0, 0, 0,
									  0, 0, 1.0, 0,
									  0, 0, 0, 1)
										 .finished();

// these are only for periodic status updates
static int FRAME_COUNTER = 0;
static int FRAME_RATE = 6;

// Keep track of previous pose changes.

///////////////////////
// END GLOBAL Variables

const google::protobuf::EnumDescriptor *descriptor_scene = rspub_pb::SceneRequestType_descriptor();
const google::protobuf::EnumDescriptor *descriptor_data = rspub_pb::DataType_descriptor();

using TSDFVolumeColorType = open3d::pipelines::integration::TSDFVolumeColorType;
using STSDFVolume = open3d::pipelines::integration::ScalableTSDFVolume;
namespace rspub
{

enum State
{
	STOP,
	START
};
struct Scene
{
	std::string scene_name;
	std::shared_ptr<STSDFVolume> volume;
	State state;
};

Eigen::Matrix4d make_transform(rspub_pb::Vec4 &rotation, rspub_pb::Vec3 &trans)
{
	Eigen::Matrix3d mat3 = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z()).toRotationMatrix();
	Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
	mat4.block(0, 0, 3, 3) = mat3;
	mat4(0, 3) = trans.x();
	mat4(1, 3) = trans.y();
	mat4(2, 3) = trans.z();
	return mat4;
}

o3dGeom::Image *createImage(const char *data, int width, int height, int num_of_channels, int bytes_per_channel)
{
	auto img = new o3dGeom::Image();
	img->Prepare(width, height, num_of_channels, bytes_per_channel);
	memcpy(img->data_.data(), data, img->data_.size());
	return img;
}

// PingService implementation
class IntegrateServiceImpl : public rspub_pb::IntegrateService
{
public:
	IntegrateServiceImpl(toml::value tcf) : tcf(tcf)
	{

		// Set member variables from configuration file
		min_translate_change = toml::find_or<double>(tcf, "min_translate_change", 0.0); // meters
		min_rotation_change = toml::find_or<double>(tcf, "min_rotation_change", 0.0);   // degrees
		min_rotation_change = (1.0 - cos(degreesToRadians(min_rotation_change))) / 2.0; // number between 0-1
		// Depth parameters
		depth_trunc = toml::find_or<double>(tcf, "depth_trunc", 3.0);
		depth_scale = toml::find_or<double>(tcf, "depth_scale", 1000.0);
		// auto intrinsic_fpath = toml::find_or<std::string>(tcf, "path_intrinsic", "");
		// open3d::io::ReadIJsonConvertible(intrinsic_fpath, camera_intrinsic);

		// std::cout << camera_intrinsic.GetPrincipalPoint().first << std::endl;
		// std::cout << "No crash yet" << std::endl;
		// Application Parameters
		auto_start = toml::find_or<bool>(tcf, "auto_start", true);
		save_dir = toml::find_or<std::string>(tcf, "save_dir", "data");
		// Mesh Parameters
		color_vertices = toml::find_or<bool>(tcf, "color_vertices", true);
		// create subscriber
		sub_rgbd = rspub::SubImage("RGBDMessage");
		// auto rgbd_rec_callback = std::bind(this->OnRGBDMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
		// sub_rgbd.AddReceiveCallback(rgbd_rec_callback);
		sub_rgbd.AddReceiveCallback([this](const char *topic_name_, const rspub_pb::ImageMessage &im_msg, const long long time_, const long long clock_, const long long stuff_) {
			OnRGBDMessage(topic_name_, im_msg, time_, clock_);
		});

		if (auto_start)
		{
			add_scene("Default");
			start_scene("Default");
			LOG(INFO) << "Auto Start Enabled: Creating \"Default\" scene and starting integration";
		}
	}

	/**
	 * @brief Creates a TSDFVolume to be intergrated
	 * 
	 * @param scene_name 
	 */
	bool add_scene(std::string scene_name)
	{
		auto it = scene_map.find(scene_name);
		if (it != scene_map.end())
		{
			return false;
		}
		auto tsdf = toml::find(tcf, "scalable_tsdf");
		double voxel_length = toml::find_or<double>(tsdf, "voxel_length", 0.03125);
		double sdf_trunc = toml::find_or<double>(tsdf, "sdf_trunc", 0.08);
		TSDFVolumeColorType color_type = TSDFVolumeColorType(toml::find_or<int>(tsdf, "color_type", 1));
		int depth_sampling_stride = toml::find_or<int>(tsdf, "depth_sampling_stride", 4);
		int volume_unit_resolution = 16;

		scene_names.insert(scene_name);
		auto ptr = std::make_shared<STSDFVolume>(voxel_length, sdf_trunc, color_type, volume_unit_resolution, depth_sampling_stride);

		Scene scene_;
		scene_.scene_name = scene_name;
		scene_.volume = ptr;
		scene_.state = State::STOP;

		scene_map[scene_name] = scene_;

		return true;
	}
	bool remove_scene(std::string scene_name)
	{
		auto it = scene_map.find(scene_name);
		if (it == scene_map.end())
		{
			return false;
		}
		scene_map.erase(it, scene_map.end());
		scene_names.erase(scene_name);
		return true;
	}
	bool start_scene(std::string scene_name)
	{
		auto it = scene_map.find(scene_name);
		if (it == scene_map.end())
		{
			add_scene(scene_name);
		}
		auto &scene = scene_map[scene_name];
		scene.state = State::START;
		return true;
	}
	bool stop_scene(std::string scene_name)
	{
		auto it = scene_map.find(scene_name);
		if (it == scene_map.end())
		{
			return false;
		}
		auto &scene = scene_map[scene_name];
		scene.state = State::STOP;
		return true;
	}
	void IntegrateScene(::google::protobuf::RpcController * /* controller_ */, const rspub_pb::IntegrateRequest *request_, rspub_pb::IntegrateResponse *response_, ::google::protobuf::Closure * /* done_ */)
	{
		// print request
		LOG(INFO) << "Received Integrate Scene Request " << descriptor_scene->FindValueByNumber(request_->type())->name()
				  << ": " << request_->scene();

		switch (request_->type())
		{
		case rspub_pb::SceneRequestType::ADD:
		{
			// LOG(INFO) << "Got Add";
			auto success = add_scene(request_->scene());
			response_->set_success(success);
			if (!success)
			{
				response_->set_message("Scene Already Exists");
			}
			break;
		}
		case rspub_pb::SceneRequestType::REMOVE:
		{
			// LOG(INFO) << "Got Remove";
			auto success = remove_scene(request_->scene());
			if (!success)
			{
				response_->set_message("DOES NOT EXIST");
			}
			response_->set_success(success);
			break;
		}
		case rspub_pb::SceneRequestType::START:
		{
			// LOG(INFO) << "Got Start";
			auto success = start_scene(request_->scene());
			response_->set_success(success);
			break;
		}
		case rspub_pb::SceneRequestType::STOP:
		{
			// LOG(INFO) << "Got Stop";
			auto success = stop_scene(request_->scene());
			response_->set_success(success);
			break;
		}

		default:
			break;
		}

		UpdatePreAndPostMatrices(request_);
	}

	void UpdatePreAndPostMatrices(const rspub_pb::IntegrateRequest *request_)
	{
		// auto pre_multiply_rf = request_->pre_multiply();
		if (request_->pre_multiply().data_size() > 0)
		{
			LOG(INFO) << "Updating Pre Multiply Matrix";
			auto pre_multiply_message = request_->pre_multiply();
			set_matrix(pre_multiply, pre_multiply_message);
		}
		if (request_->post_multiply().data_size() > 0)
		{
			LOG(INFO) << "Updating Post Multiply Matrix";
			auto post_multiply_message = request_->post_multiply();
			set_matrix(post_multiply, post_multiply_message);
		}

		LOG(INFO) << "Premultiply matrix: " << pre_multiply;
		LOG(INFO) << "PostMultiply matrix: " << post_multiply;
	}

	void set_matrix(Eigen::Matrix4d &eigen_matrix, rspub_pb::DoubleMatrix &matrix_message)
	{
			eigen_matrix(0, 0) = matrix_message.data(0);
			eigen_matrix(0, 1) = matrix_message.data(1);
			eigen_matrix(0, 2) = matrix_message.data(2);
			eigen_matrix(0, 3) = matrix_message.data(3);

			eigen_matrix(1, 0) = matrix_message.data(4);
			eigen_matrix(1, 1) = matrix_message.data(5);
			eigen_matrix(1, 2) = matrix_message.data(6);
			eigen_matrix(1, 3) = matrix_message.data(7);

			eigen_matrix(2, 0) = matrix_message.data(8);
			eigen_matrix(2, 1) = matrix_message.data(9);
			eigen_matrix(2, 2) = matrix_message.data(10);
			eigen_matrix(2, 3) = matrix_message.data(11);

			eigen_matrix(3, 0) = matrix_message.data(12);
			eigen_matrix(3, 1) = matrix_message.data(13);
			eigen_matrix(3, 2) = matrix_message.data(14);
			eigen_matrix(3, 3) = matrix_message.data(15);
	}


	bool SaveMesh(std::string scene_name)
	{
		std::string fpath = save_dir + "/" + scene_name + ".ply";
		auto mesh = GetMesh(scene_name);
		if (mesh)
		{
			o3d::io::WriteTriangleMesh(fpath, *mesh.get());
			return true;
		}
		return false;
	}

	std::shared_ptr<open3d::geometry::TriangleMesh> GetMesh(std::string scene_name)
	{
		auto it = scene_map.find(scene_name);
		if (it == scene_map.end())
		{
			return nullptr;
		}
		auto &volume = it->second.volume;
		auto mesh = volume->ExtractTriangleMesh();
		return mesh;
	}

	bool ExtractMesh(std::string scene_name, rspub_pb::Mesh &mesh_pb)
	{
		double now0 = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
		auto mesh = GetMesh(scene_name);
		double now1 = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
		if (!mesh)
			return false;
		// auto halfedges = o3d::geometry::HalfEdgeTriangleMesh::ExtractHalfEdges(*mesh.get());
		double now2 = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();

		int n_triangles = mesh->triangles_.size();
		int n_vertices = mesh->vertices_.size();
		int nbytes_triangles = n_triangles * 4 * 3; // 4 bytes per vertex index, 3 vertex indices per triangle
		int nbytes_halfedges = n_triangles * 8 * 3; // 8 bytes per half edge id (uint64), 3 edges per triangle
		int nbytes_vertices = n_vertices * 8 * 3;   // 8 bytes per point (double), 3 points per vertex.
		mesh_pb.set_vertices(mesh->vertices_.data(), nbytes_vertices);
		mesh_pb.set_triangles(mesh->triangles_.data(), nbytes_triangles);
		// mesh_pb.set_halfedges(halfedges.data(), nbytes_halfedges);
		mesh_pb.set_n_triangles(n_triangles);
		mesh_pb.set_n_vertices(n_vertices);
		if (color_vertices)
		{
			mesh_pb.set_vertices_colors(mesh->vertex_colors_.data(), nbytes_vertices);
		}
		double now3 = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
		LOG(INFO) << std::setprecision(3) << std::fixed << "Mesh Extraction took: " << now1 - now0 << "; Half Edge Extraction: " << now2 - now1 << "; Serialization: " << now3 - now2;
		VLOG(2) << "Vertices: " << n_vertices << "; Triangles: " << n_triangles;
		return true;
	}

	void ExtractScene(::google::protobuf::RpcController * /* controller_ */, const rspub_pb::ExtractRequest *request_, rspub_pb::ExtractResponse *response_, ::google::protobuf::Closure * /* done_ */)
	{
		// print request
		LOG(INFO) << "Received Extract Request " << descriptor_data->FindValueByNumber(request_->type())->name()
				  << ": " << request_->scene();

		switch (request_->type())
		{
		case rspub_pb::DataType::MESH:
		{
			// LOG(INFO) << "Got Mesh Request";
			response_->set_type(rspub_pb::DataType::MESH);
			auto mesh = response_->mutable_mesh();
			auto success = ExtractMesh(request_->scene(), *mesh);
			response_->set_success(success);
			break;
		}
		case rspub_pb::DataType::POLYGONS:
		{
			// LOG(INFO) << "Got Polygon";
			break;
		}
		case rspub_pb::DataType::POINTCLOUD:
		{
			// LOG(INFO) << "Got Pointcloud";
			break;
		}
		default:
			break;
		}
	}

	// https://math.stackexchange.com/questions/90081/quaternion-distance
	bool checkPoseChange(rspub_pb::Vec4 &rotation, rspub_pb::Vec3 &trans)
	{
		Eigen::Vector3d current_trans(trans.x(), trans.y(), trans.z());
		Eigen::Vector4d current_rot(rotation.x(), rotation.y(), rotation.z(), rotation.w());
		VLOG(3) << "Previous Translation: " << previous_trans << std::endl;
		VLOG(3) << "Current Translation: " << current_trans << std::endl;
		VLOG(3) << "Previous Rotation: " << previous_rot << std::endl;
		VLOG(3) << "Current Rotation: " << current_rot << std::endl;
		auto pose_diff = (current_trans - previous_trans).norm();
		double quat_changed = previous_rot.transpose() * current_rot;
		quat_changed = quat_changed * quat_changed;
		quat_changed = 1.0 - quat_changed;

		bool poseChanged = (pose_diff > min_translate_change || quat_changed > min_rotation_change);
		if (poseChanged)
		{
			previous_rot = current_rot;
			previous_trans = current_trans;
		}
		return poseChanged;
	}

	void OnRGBDMessage(const char *topic_name_, const rspub_pb::ImageMessage &im_msg, const long long time_, const long long clock_)
	{
		// std::cout << "OnRGBDMessage" << std::endl;
		FRAME_COUNTER += 1;
		double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
		VLOG(2) << std::setprecision(0) << std::fixed << "Received RGBDMessage; now: " << now << "; send_ts: " << time_ / 1000 << "; hardware_ts: " << im_msg.hardware_ts() << std::endl;

		auto w = im_msg.width();
		auto h = im_msg.height();
		auto image_data_str = im_msg.image_data();
		const char *image_data_ptr = image_data_str.c_str();

		auto image_data_second_str = im_msg.image_data_second();
		const char *image_data_second_ptr = image_data_second_str.c_str();

		auto rotation = im_msg.rotation();
		auto translation = im_msg.translation();

		bool poseChanged = checkPoseChange(rotation, translation);

		bool one_scene_active = false;
		for (auto &scene_name : scene_names)
		{
			auto &scene = scene_map[scene_name];
			one_scene_active = one_scene_active || scene.state == State::START;
		}


		if (FRAME_COUNTER % FRAME_RATE == 0)
		{
			std::cout << "Frame Count: " << FRAME_COUNTER << "; Active Scene: " << one_scene_active << std::endl;
		}

		if (!poseChanged || !one_scene_active)
			return;

		VLOG(1) << "Pose Changed";
		auto H_t265_W = make_transform(rotation, translation);
		// Eigen::Matrix4d extrinsic = (H_t265_d400.inverse() * H_t265_W * H_t265_d400).inverse();
		Eigen::Matrix4d extrinsic = (pre_multiply * H_t265_W * post_multiply).inverse();

		// std::cout << "About to Create Image" << std::endl;
		// Unfortunately these are making copies
		// I dont think theres anyway to avoid this with O3D API
		auto color_image = createImage(image_data_ptr, w, h, 3, 1);
		auto depth_image = createImage(image_data_second_ptr, w, h, 1, 2);

		// o3d::io::WriteImage("blah_color.png", *color_image);
		// o3d::io::WriteImage("blah_depth.png", *depth_image);
		// std::cout << "Created Image" << std::endl;
		// I think this may even be a copy!
		auto rgbd_image = o3dGeom::RGBDImage::CreateFromColorAndDepth(*color_image, *depth_image,
																	  depth_scale = depth_scale, depth_trunc = depth_trunc, false);
		// std::cout << "Here is extrinsics" << extrinsic << std::endl;
		// std::cout << "Created RGBD Image" << std::endl;
		for (auto &scene_name : scene_names)
		{
			VLOG(3) << "Looking at " << scene_name;
			auto &scene = scene_map[scene_name];
			if (scene.state == State::STOP)
				continue;
			VLOG(2) << "Integrating " << scene_name;
			double now0 = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
			o3d::camera::PinholeCameraIntrinsic camera_intrinsic(w, h, im_msg.fx(), im_msg.fy(), im_msg.cx(), im_msg.cy());
			scene.volume->Integrate(*rgbd_image, camera_intrinsic, extrinsic);
			double now1 = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
			LOG(INFO) << std::setprecision(3) << std::fixed << "Volume Integration took: " << now1 - now0 << " milliseconds";
		}
	}

protected:
	std::shared_ptr<open3d::visualization::Visualizer> vis;
	std::map<std::string, Scene> scene_map;
	std::set<std::string> scene_names;
	rspub::SubImage sub_rgbd;
	toml::value tcf;
	Eigen::Vector3d previous_trans;
	Eigen::Vector4d previous_rot;
	double depth_trunc;
	double depth_scale;
	double min_translate_change;
	double min_rotation_change;
	bool auto_start;
	bool color_vertices;
	std::string save_dir;
	// o3d::camera::PinholeCameraIntrinsic camera_intrinsic;
};

} // namespace rspub

int main(int argc, char *argv[]) try
{
	std::cout << "Starting up in rs-integrate-server" << std::endl;
	// Parse command line flags
	google::InitGoogleLogging(argv[0]);
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	LOG(INFO) << "Starting rs-integrate-server";
	// ECAL Configuration
	std::vector<std::string> ecal_args;
	ecal_args.push_back("--ecal-ini-file");
	ecal_args.push_back("./config/ecal/ecal.ini");
	if (FLAGS_force_udp)
	{
		LOG(INFO) << "Forcing UDP Multicast for ECAL";
		ecal_args.push_back("--set_config_key");
		ecal_args.push_back("publisher/use_udp_mc:1");

		ecal_args.push_back("--set_config_key");
		ecal_args.push_back("publisher/use_shm:0");
	}
	// initialize eCAL API
	eCAL::Initialize(ecal_args, "RSIntegrate");
	// Read our application configuration file
	const auto tcf = toml::parse(FLAGS_config);
	// create IntegrationServer server
	std::shared_ptr<rspub::IntegrateServiceImpl> integrate_service_impl = std::make_shared<rspub::IntegrateServiceImpl>(tcf);
	eCAL::protobuf::CServiceServer<rspub_pb::IntegrateService> integrate_service(integrate_service_impl);

	std::string save_scene;
	LOG(INFO) << "Waiting for service requests: ";
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
