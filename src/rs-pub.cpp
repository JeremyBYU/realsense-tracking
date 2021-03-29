// License: MIT. See LICENSE file in root directory.
// Copyright(c) 2019 Jeremy Castagno. All Rights Reserved.
// Realsense Publish Module

// This technique works with RSUSB driver (LIBUVC) but should work with kernel
// drivers as well This example is meant for D435i, to get high frequency motion
// data at the same time of depth,rgb images Two examples are shown, one that
// read a bags file and another that does live streaming from the sensor.
// Activate bag reading with cmd line flag --bag=<file> The main technique that
// makes this possible is to NOT USE rs::pipeline, but to directly access each
// sensor and configure the stream manually (as well as associated callbacks).

#include <chrono>
#include <map>
#include <string>
#include <thread>
// #include <memory>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <librealsense2/rs.hpp>
// #include <opencv2/opencv.hpp>

#include "rspub/utility.hpp"

using namespace std::chrono_literals;
using namespace std::string_literals;

// Command line flags
DEFINE_string(bag, "", "Path to bag file");
DEFINE_string(config, "./config/rspub_default.toml", "Path to config file");
DEFINE_bool(force_udp, false, "Force UDP Multicast for publishers");

rspub::RingBuffer<rspub::TransformTS> POSES(100);
std::mutex POSES_MUTEX;

// GLOBAL counters for t265 and D4XX/L515
int POSE_COUNTER = 0;
int FRAME_COUNTER = 0;
const int POSE_RATE = 200;
const int FRAME_RATE = 30;


namespace rspub {

// typedef std::map<std::string, std::shared_ptr<eCAL::CPublisher>>
// PublisherMap;

void rs_callback(rs2::frame &frame,
                 eCAL::protobuf::CPublisher<rspub_pb::PoseMessage> *pose_pub) {
  if (frame.get_profile().stream_type() == RS2_STREAM_GYRO) {
  }
  if (frame.get_profile().stream_type() == RS2_STREAM_DEPTH) {
    std::cout << "Got Depth" << std::endl;
  } else if (frame.get_profile().stream_type() == RS2_STREAM_ACCEL) {
  } else if (frame.get_profile().stream_type() == RS2_STREAM_POSE) {
    POSE_COUNTER += 1;
    if (POSE_COUNTER % POSE_RATE)
    {
      std::cout << "Received Poses; Pose Counter: " << POSE_COUNTER << std::endl;
    }
    auto pframe = frame.as<rs2::pose_frame>();
    rs2_pose pose = pframe.get_pose_data();
    // std::cout << std::setprecision(0) << std::fixed << std::left <<
    // std::setw(11) << "POSE: " << frame.get_timestamp() << std::endl;
    rspub_pb::PoseMessage pose_message;
    fill_pose_message(pose, pose_message, frame.get_timestamp());
    pose_pub->Send(pose_message);

    // Push only TRANSFORM information into a 100 size ringbuffer
    std::lock_guard<std::mutex> lockGuard(POSES_MUTEX);
    TransformTS transform_ts = {
        {pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w},
        {pose.translation.x, pose.translation.y, pose.translation.z},
        frame.get_timestamp()};
    POSES.push(transform_ts);
  }
}

/**
 * @brief Enable a device stream (manual). It will go through all the sensors on
 * the device and "open" them for streaming with an attached callback This
 * method of interacting is (was?) necessary so that high frequency data
 * (Accelerometer) can be (**reliably** pulled out at the same time as low
 * frequency data (images). Using a pipeline did not work for me. This is a very
 * manual/heavy way to get the data, but it works.
 *
 * @param dev 					The device of intereste
 * @param desired_devices 		The string vector of all desired devices
 * @param ctx 					The RealSense Context
 * @param desired_streams 		A list of all desired streams for manual
 * streaming
 * @param device_sensors 		All the device sensors
 * @param callback 				The callback for processing
 * frames
 */
void enable_device_stream(
    rs2::device &dev, std::vector<std::string> &desired_devices,
    rs2::context &ctx, std::vector<rspub::StreamDetail> &desired_streams,
    std::map<std::string, std::vector<rs2::sensor>> &device_sensors,
    std::function<void(rs2::frame)> callback) {
  auto name = std::string(dev.get_info(RS2_CAMERA_INFO_NAME));
  if (std::find(desired_devices.begin(), desired_devices.end(), name) ==
      desired_devices.end())
    return;
  // The sensors **have** to live outside of the device for loop! Therfore put
  // inside of the std::map device_sensors
  device_sensors[name] = dev.query_sensors();
  LOG(INFO) << "Attempting to open " << name << ": "
            << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
  for (auto &sensor : device_sensors[name]) {
    auto sensor_name = std::string(sensor.get_info(RS2_CAMERA_INFO_NAME));
    LOG(INFO) << "Sensor Name: " << sensor_name << std::endl;
    try {
      auto sensor_streams = sensor.get_stream_profiles();
      // rstracker::print_profiles(sensor_streams);
      std::vector<rs2::stream_profile> allowed_streams;
      // get desired stream profiles objects
      for (auto &sp : sensor_streams) {
        auto sd_ = rspub::stream_profile_to_details(sp);
        if (std::find(desired_streams.begin(), desired_streams.end(), sd_) !=
            desired_streams.end()) {
          allowed_streams.push_back(sp);
        }
      }

      if (allowed_streams.size() > 0) {
        // Open Sensor for streaming
        LOG(INFO) << "Opening stream for " << sensor_name << std::endl;
        sensor.open(allowed_streams);
        // Sensor Callback
        LOG(INFO) << "Creating callback for " << sensor_name << " streams"
                  << std::endl;
        // sensor.start([&](rs2::frame frame) { LOG(INFO) << "Inside sensor
        // lambda"; callback(&frame);LOG(INFO) << "After callback";  });
        sensor.start(callback);
      }
    } catch (const std::exception &e) {
      LOG(INFO) << "Sensor " << sensor_name
                << " has not configured streams, skipping..." << e.what()
                << std::endl;
    }
  }
}

/**
 * @brief This will enable manual streaming. In other words a specifc stream
 * will have its own isolated callback for processing frames (data)
 *
 * @param ctx 							The RealSense
 * Context
 * @param desired_streams 				A list of all desired
 * manual streams
 * @param device_sensors 				All device sensors
 * @param callback 						The function callback
 * for processing a RealSense Frame
 * @param single_device 				If this is a single
 * device (RealSense bag file only has one 'fake' device)
 */
void enable_manual_streams(
    rs2::context &ctx, std::vector<rspub::StreamDetail> &desired_streams,
    std::map<std::string, std::vector<rs2::sensor>> &device_sensors,
    std::function<void(rs2::frame)> callback,
    rs2::device *single_device = nullptr) {
  std::vector<std::string> desired_devices;
  std::transform(desired_streams.begin(), desired_streams.end(),
                 std::back_inserter(desired_devices),
                 [](auto const &sd) { return sd.device_name; });

  LOG(INFO) << "Requesting to start manual streams: " << std::endl;
  rspub::print_profiles(desired_streams);

  std::set<std::string> all_devices;

  if (single_device) {
    enable_device_stream(*single_device, desired_devices, ctx, desired_streams,
                         device_sensors, callback);
  } else {
    auto devices = ctx.query_devices();
    LOG(INFO) << "Devices available: " << std::endl;
    for (auto dev_ : devices) {
      auto name = std::string(dev_.get_info(RS2_CAMERA_INFO_NAME));
      all_devices.insert(name);
      LOG(INFO) << "Devices is: " << name << ": "
                << dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    }
    for (auto dev : devices) {
      enable_device_stream(dev, desired_devices, ctx, desired_streams,
                           device_sensors, callback);
    }
  }
  for (auto &desired_device : desired_devices) {
    const bool is_in = all_devices.find(desired_device) != all_devices.end();
    if (!is_in) {
      LOG(ERROR) << "ERROR! Asked for " << desired_device
                 << " but not availble";
    }
  }
}

/**
 * @brief Configures and starts a pipeline for the desired streams. Only works
 * for one device. (e.g. D4XX)
 *
 * @param desired_pipeline_streams 		List of desired streams in the
 * pipeline
 * @param cfg 							RealSense
 * configuration
 * @param pipe 							RealSense
 * Pipeline object
 * @param tcf 							TOML config file
 */
void enable_pipe_streams(
    std::vector<rspub::StreamDetail> &desired_pipeline_streams,
    rs2::config &cfg, rs2::pipeline &pipe, const toml::value &tcf) {
  LOG(INFO) << "Requesting to start pipeline streams: " << std::endl;
  rspub::print_profiles(desired_pipeline_streams);
  for (auto &pipe_stream : desired_pipeline_streams) {
    cfg.enable_stream(pipe_stream.stream_type, pipe_stream.width,
                      pipe_stream.height, pipe_stream.format, pipe_stream.fps);
  }
  // Start streaming with default recommended configuration
  auto profile = pipe.start(cfg);
  auto sensor = profile.get_device().first<rs2::depth_sensor>();
  if (sensor) {
    auto auto_exposure = toml::find_or<float>(tcf, "depth_auto_exposure", 1);
    auto exposure = toml::find_or<float>(tcf, "depth_exposure", 1);
    if (auto_exposure == 0.0) {
      sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure);
      sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
      LOG(INFO) << "Setting exposure to: " << exposure << " microseconds";
    }

    auto global_time_enabled =
        toml::find_or<float>(tcf, "global_time_enabled", 0);
    if (global_time_enabled) {
      sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, global_time_enabled);
      LOG(INFO) << "Enabling Global time on depth sensor";
    }
  }

  auto sensor_color = profile.get_device().first<rs2::color_sensor>();
  if (sensor_color) {
    auto global_time_enabled =
        toml::find_or<float>(tcf, "global_time_enabled", 0);
    if (global_time_enabled) {
      sensor_color.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED,
                              global_time_enabled);
      LOG(INFO) << "Enabling Global time on color sensor";
    }
  }
}

/**
 * @brief This will start an already created RealSense pipeline given the
 * desired streams and settings in a config file The data wll be published on
 * separate topics
 *
 * @param dsp 				A vector of all streams
 * @param pipe 				The realsense pipeline
 * @param tcf 				The parsed toml config file
 * @param pub_depth 		The Ecal publisher for depth images
 * @param pub_color 		The ecal publicsher for color images
 * @param pub_rgbd 			The ecal publisher for RGBD images
 * @param pub_pc 			The ecal publisher for point clouds
 * @param wait 				Should we wait between frames. Deault is
 * no waiting. Useful for pseudo-streaming from bag file.
 */
void process_pipeline(std::vector<rspub::StreamDetail> dsp, rs2::pipeline &pipe,
                      const toml::value &tcf, PubImage &pub_depth,
                      PubImage &pub_color, PubImage &pub_rgbd,
                      PubPointCloud &pub_pc, bool wait = false) {

  // create filters for depth image
  std::vector<NamedFilter> filters;
  const std::tuple<bool, std::string> align = create_filters(filters, tcf);
  auto stream_to_align =
      std::get<1>(align) == "color" ? RS2_STREAM_COLOR : RS2_STREAM_DEPTH;
  rs2::align align_to_color(
      stream_to_align); // when aligining to color, no distortion in
                        // coeffecients when aligining to depth, no distortion
                        // in coeffecients
  auto depth_cfg = toml::find(tcf, "publish", "depth");
  auto depth_cfg_active = toml::find_or<bool>(depth_cfg, "active", false);
  int depth_cfg_rate = toml::find_or<int>(depth_cfg, "rate", 1);
  int depth_cfg_counter = 0;

  auto color_cfg = toml::find(tcf, "publish", "color");
  auto color_cfg_active = toml::find_or<bool>(color_cfg, "active", false);
  int color_cfg_rate = toml::find_or<int>(color_cfg, "rate", 1);
  int color_cfg_counter = 0;

  auto rgbd_cfg = toml::find<toml::value>(tcf, "publish", "rgbd");
  auto rgbd_cfg_active = toml::find_or<bool>(rgbd_cfg, "active", false);
  int rgbd_cfg_rate = toml::find_or<int>(rgbd_cfg, "rate", 1);
  int rgbd_cfg_counter = 0;

  auto pc_cfg = toml::find(tcf, "publish", "pointcloud");
  bool pc_cfg_active = toml::find_or<bool>(pc_cfg, "active", false);
  bool pc_cfg_color = toml::find_or<bool>(pc_cfg, "color", false);
  int pc_cfg_stride = toml::find_or<int>(pc_cfg, "stride", 1);
  int pc_cfg_rate = toml::find_or<int>(pc_cfg, "rate", 1);
  int pc_cfg_counter = 0;

  rs2::pointcloud pc;
  rs2::points points;

  // auto stream_profile = depth.get_profile();

  if (dsp.size() > 0) {
    while (true) {
      depth_cfg_counter++;
      color_cfg_counter++;
      pc_cfg_counter++;
      rgbd_cfg_counter++;
      FRAME_COUNTER++; // global counter for all frames, every iteration of this while loop

      auto frames = pipe.wait_for_frames();

      auto dframe = frames.get_depth_frame();
      auto cframe = frames.get_color_frame();

      double dts = 0;
      double cts = 0;

      double now = std::chrono::duration<double, std::milli>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
      auto start_cwidth = cframe ? cframe.get_width() : 0;
      VLOG(1) << std::setprecision(0) << std::fixed
              << "Received FrameSet; now: " << now
              << "; dwidth: " << dframe.get_width()
              << "; cwidth: " << start_cwidth;
      // continue; // LOG(INFO) << std::setprecision(0) << std::fixed <<
      // "Received FrameSet; now: " << now << "; cwidth: " <<
      // cframe.get_width();
      if (dframe) {
        dts = dframe.get_timestamp();
        VLOG(2) << "num_filters: " << static_cast<int>(filters.size());
        for (std::vector<NamedFilter>::const_iterator filter_it =
                 filters.begin();
             filter_it != filters.end(); filter_it++) {
          VLOG(2) << "Applying filter: " << filter_it->_name;
          auto t0 = std::chrono::high_resolution_clock::now();
          frames = filter_it->_filter->process(frames);
          auto t1 = std::chrono::high_resolution_clock::now();
          float elapsed_d =
              static_cast<std::chrono::duration<float, std::milli>>(t1 - t0)
                  .count();
          // LOG(INFO) << std::setprecision(3) << std::fixed << "Filter " <<
          // filter_it->_name << " took: " << elapsed_d;
        }
        dframe = frames.get_depth_frame();
      }

      if (dframe && cframe && std::get<0>(align)) {
        VLOG(2) << "Applying filter: align";
        frames = align_to_color.process(frames);
        dframe = frames.get_depth_frame();
        cframe = frames.get_color_frame();
      }

      VLOG(2) << "Filtering Complete";
      // std::cout << "Before depth check: " << depth_cfg_active << " " <<
      // depth_cfg_counter << " " << depth_cfg_rate << std::endl;
      if (dframe && depth_cfg_active && depth_cfg_counter == depth_cfg_rate) {
        // std::cout << "Inside to publish depth" << std::endl;
        VLOG(2) << "Publishing dframe";
        rspub_pb::ImageMessage depth_message;
        fill_image_message(dframe, depth_message);
        pub_depth.Send(depth_message);
        depth_cfg_counter = 0;
      }

      if (cframe && color_cfg_active && color_cfg_counter == color_cfg_rate) {
        VLOG(2) << "Publishing cframe";
        rspub_pb::ImageMessage color_message;
        fill_image_message(cframe, color_message);
        pub_color.Send(color_message);
        color_cfg_counter = 0;
        cts = cframe.get_timestamp();
      }

      if (cframe && dframe && rgbd_cfg_active &&
          rgbd_cfg_counter == rgbd_cfg_rate) {
        VLOG(2) << "Publishing rgbd frame";
        rspub_pb::ImageMessage rgbd_message;
        fill_image_message(cframe, rgbd_message);
        fill_image_message_second(dframe, rgbd_message);
        if (POSES.sz > 0 && POSES.full()) {
          std::lock_guard<std::mutex> lockGuard(POSES_MUTEX);
          double ts_merged = dframe.get_timestamp();
          auto transform = getNearestTransformTS(ts_merged, POSES);
          fill_image_message_transform(transform, rgbd_message);
          auto diff = std::abs(transform.ts - ts_merged);
          if (FRAME_COUNTER % FRAME_RATE == 0)
          {
            std::cout << "Frame Count: " << FRAME_COUNTER << "; Pose timestamp diff (ms): " << diff;
          }
          
          if (diff > static_cast<double>(POSE_RATE)) {
            LOG(ERROR) << "RGBD and Pose have drifted more than 1 second from eachother!";
            std::cout << "RGBD and Pose have drifted more than 1 second from eachother!" << std::endl;
            throw std::runtime_error("RGBD and Pose data out of sync");
          }
          // LOG(INFO) << std::setprecision(3) << std::fixed <<  "Current
          // Timestamp: " << ts_merged << "; transform Timestamp: " <<
          // transform.ts << "; translation x: " << transform.pos[0];
        }
        pub_rgbd.Send(rgbd_message);
        rgbd_cfg_counter = 0;
      }

      start_cwidth = cframe ? cframe.get_width() : 0;
      now = std::chrono::duration<double, std::milli>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
      VLOG(1) << std::setprecision(0) << std::fixed
              << "End of FrameSet Loop; now: " << now
              << "; dframe hardware_ts: " << dts
              << "; cframe hardware_ts: " << cts
              << "; dwidth: " << dframe.get_width()
              << "; cwidth: " << start_cwidth;
      if (dframe && pc_cfg_active && pc_cfg_counter == pc_cfg_rate) {
        VLOG(2) << "Publishing pointcloud";
        rspub_pb::PointCloudMessage pc_message;
        double now0 = std::chrono::duration<double, std::micro>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
        fill_pointcloud_message_optimized(dframe, cframe, pc_message,
                                          dframe.get_timestamp(), pc_cfg_stride,
                                          pc_cfg_color);
        // fill_pointcloud(dframe, cframe, pc, points, pc_cfg_color);
        // fill_pointcloud_message(points, cframe, pc_message,
        // dframe.get_timestamp(), pc_cfg_color);
        double now1 = std::chrono::duration<double, std::micro>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
        pub_pc.Send(pc_message);
        double now2 = std::chrono::duration<double, std::micro>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
        pc_cfg_counter = 0;
        ;
        VLOG(2) << std::setprecision(0) << std::fixed
                << "Create Point Cloud and Fill Message: " << now1 - now0
                << "; Send Point Cloud: " << now2 - now1;
      }

      if (wait)
        std::this_thread::sleep_for(1000us);
    }
  } else {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
}

/**
 * @brief This function will read a RealSense Bag file and begin publishing the
 * data
 *
 * @param file_name 			File name
 * @param tcf 					Configuration for toml file
 * @return int
 */
int read_bag(std::string file_name, const toml::value &tcf) {
  // I'm not sure if this works method works full proof for bag files.
  LOG(INFO) << "Creating BagReader of RealSense" << std::endl;
  // create a publisher (topic name "RSPub")
  eCAL::Initialize(0, nullptr, "RSPub");
  // std::make_unique<eCAL::protobuf::CPublisher<rspub_pb::PoseMessage>>("PoseMessage");
  PubPose pub_pose("PoseMessage");
  PubImage pub_depth("DepthMessage");
  PubImage pub_color("ColorMessage");
  PubImage pub_rbgd("RGBDMessage");
  PubPointCloud pub_pc("PointCloudMessage");

  // Create librealsense context for managing devices
  rs2::context ctx;
  // Desired streams that should be manually controlled (no pipeline, i.e.,
  // sensor.open)
  std::vector<rspub::StreamDetail> desired_manual_streams;
  // Desired streams that should be controlled and SYNCED through a pipeline.
  // Like Depth and Color.
  std::vector<rspub::StreamDetail> desired_pipeline_streams;
  // Read from TOML Config file
  parse_desired_stream(desired_manual_streams, tcf, "manual_streams");
  parse_desired_stream(desired_pipeline_streams, tcf, "pipeline_streams");
  // std::vector<rspub::StreamDetail> desired_manual_streams = {{"Intel
  // RealSense T265", "Pose", rspub::STRM_ENUM.at("Pose"s), 0, 0, 200,
  // rspub::FMT_ENUM.at("6DOF"s)}}; std::vector<rspub::StreamDetail>
  // desired_pipeline_streams = {{"Intel RealSense D435I", "Depth",
  // rspub::STRM_ENUM.at("Depth"s), 640, 480, 30, rspub::FMT_ENUM.at("Z16"s)},
  // 																 {"Intel RealSense
  // D435I", "Color", rspub::STRM_ENUM.at("Color"s), 640, 480, 30,
  // rspub::FMT_ENUM.at("BGR8"s)}};

  // needed this variable to keep sensors 'alive'
  std::map<std::string, std::vector<rs2::sensor>> device_sensors;
  auto pose_rec_callback = [&pub_pose](rs2::frame frame) {
    rs_callback(frame, &pub_pose);
    std::this_thread::sleep_for(1000us);
  };

  rs2::config config;
  rs2::device device;
  rs2::pipeline pipe(ctx);

  // enable file playback with playback repeat disabled
  config.enable_device_from_file(file_name, false);

  auto profile = config.resolve(pipe);
  device = profile.get_device();
  auto playback = device.as<rs2::playback>();
  playback.set_real_time(false);

  if (desired_manual_streams.size() > 0) {
    enable_manual_streams(ctx, desired_manual_streams, device_sensors,
                          pose_rec_callback, &device);
  }

  if (desired_pipeline_streams.size() > 0) {
    LOG(INFO) << "Requesting to start: " << std::endl;
    print_profiles(desired_pipeline_streams);
    pipe.start(config);
  }

  process_pipeline(desired_pipeline_streams, pipe, tcf, pub_depth, pub_color,
                   pub_rbgd, pub_pc);
  return EXIT_SUCCESS;
}

/**
 * @brief This function wil initiate connections to LIVE RealSense Cameras
 *
 * @param tcf 			This is a parsed toml file for configuration
 * @return int
 */
int live_stream(const toml::value &tcf) {
  try {
    // create a publisher (topic name "RSPub")
    LOG(INFO) << "Creating LiveStream of RealSense" << std::endl;
    eCAL::Initialize(0, nullptr, "RSPub");
    PubPose pub_pose("PoseMessage");
    PubImage pub_depth("DepthMessage");
    PubImage pub_color("ColorMessage");
    PubPointCloud pub_pc("PointCloudMessage");
    PubImage pub_rgbd("RGBDMessage");
    // Create librealsense context for managing devices
    rs2::context ctx;
    // Desired streams that should be manually controlled (no pipeline, i.e.,
    // sensor.open)
    std::vector<rspub::StreamDetail> desired_manual_streams;
    // Desired streams that should be controlled and SYNCED through a pipeline.
    // Like Depth and Color.
    std::vector<rspub::StreamDetail> desired_pipeline_streams;
    // Read from TOML Config file
    parse_desired_stream(desired_manual_streams, tcf, "manual_streams");
    parse_desired_stream(desired_pipeline_streams, tcf, "pipeline_streams");

    // needed this variable to keep sensors 'alive'
    std::map<std::string, std::vector<rs2::sensor>> device_sensors;
    auto pose_rec_callback = [&pub_pose](rs2::frame frame) {
      rs_callback(frame, &pub_pose);
    };

    if (desired_manual_streams.size() > 0) {
      enable_manual_streams(ctx, desired_manual_streams, device_sensors,
                            pose_rec_callback);
    }

    rs2::config cfg;
    rs2::pipeline pipe(ctx);
    if (desired_pipeline_streams.size() > 0) {
      enable_pipe_streams(desired_pipeline_streams, cfg, pipe, tcf);
    }

    process_pipeline(desired_pipeline_streams, pipe, tcf, pub_depth, pub_color,
                     pub_rgbd, pub_pc);

    return EXIT_SUCCESS;
    /* code */
  } catch (const rs2::error &e) {

    LOG(ERROR) << "RealSense error calling " << e.get_failed_function() << "("
               << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;

    return EXIT_FAILURE;
  }

  catch (const std::exception &e) {
    LOG(ERROR) << e.what() << std::endl;
    std::cerr << e.what() << std::endl;

    return EXIT_FAILURE;
  }
}

int repeat_live_stream(const toml::value &tcf) {
  int repeat = 0;
  int max_repeat = 5;
  do {
    auto result = live_stream(tcf);
    std::cout << "Whoa we exited the live stream!" << result << std::endl;
    if (result == EXIT_FAILURE) {
      std::cout << "Failure! Will try to restart" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      repeat++;

    } else {
      std::cout << "Exit Succesfully! Won't restart" << std::endl;
      repeat = 3;
    }
  } while (repeat < max_repeat);
}

} // namespace rspub

int main(int argc, char *argv[]) try {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // ECAL Configuration
  std::vector<std::string> ecal_args;
  ecal_args.push_back("--ecal-ini-file");
  ecal_args.push_back("./config/ecal/ecal.ini");
  if (FLAGS_force_udp) {
    LOG(INFO) << "Forcing UDP Multicast for ECAL";
    ecal_args.push_back("--set_config_key");
    ecal_args.push_back("publisher/use_udp_mc:1");

    ecal_args.push_back("--set_config_key");
    ecal_args.push_back("publisher/use_shm:0");
  }
  LOG(INFO) << "Starting RealSense Publisher";
  // initialize eCAL API
  eCAL::Initialize(ecal_args, "RSPub");

  // // enable to receive process internal publications
  // eCAL::Util::EnableLoopback(true);
  // Parse command line flags
  if (FLAGS_config == "")
    LOG(ERROR) << "Must specify path to TOML config file";

  const auto tcf = toml::parse(FLAGS_config);
  std::thread t1;
  if (FLAGS_bag == "") {
    t1 = std::thread(rspub::repeat_live_stream, tcf);
  } else {
    t1 = std::thread(rspub::read_bag, FLAGS_bag, tcf);
  }

  eCAL::Process::SleepMS(1000);
  while (eCAL::Ok()) {
    eCAL::Process::SleepMS(100);
  }
} catch (const rs2::error &e) {

  LOG(ERROR) << "RealSense error calling " << e.get_failed_function() << "("
             << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;

  return EXIT_FAILURE;
}

catch (const std::exception &e) {
  LOG(ERROR) << e.what() << std::endl;
  std::cerr << e.what() << std::endl;

  return EXIT_FAILURE;
}
