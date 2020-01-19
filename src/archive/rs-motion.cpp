// License: Apache 2.0. See LICENSE file in root directory.

// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <cstring>
#include <chrono>
#include <thread>
#include <mutex>

#include <librealsense2/rs.hpp>
#include <gflags/gflags.h>



#include "example.hpp" // Include short list of convenience functions for rendering

// Command line flags

DEFINE_string(bag, "",
				"Path to bag file");

struct short3
{
	uint16_t x, y, z;
};
class rotation_estimator
{
	// theta is the angle of camera rotation in x, y and z components
	float3 theta;
	std::mutex theta_mtx;
	/* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
	values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
	float alpha = 0.98;
	bool first = true;
	// Keeps the arrival time of previous gyro frame
	double last_ts_gyro = 0;

public:
	// Function to calculate the change in angle of motion based on data from gyro
	void process_gyro(rs2_vector gyro_data, double ts)
	{
		if (first) // On the first iteration, use only data from accelerometer to set the camera's initial position
		{
			last_ts_gyro = ts;
			return;
		}
		// Holds the change in angle, as calculated from gyro
		float3 gyro_angle;
		// Initialize gyro_angle with data from gyro
		gyro_angle.x = gyro_data.x; // Pitch
		gyro_angle.y = gyro_data.y; // Yaw
		gyro_angle.z = gyro_data.z; // Roll
		// Compute the difference between arrival times of previous and current gyro frames
		double dt_gyro = (ts - last_ts_gyro) / 1000.0;
		last_ts_gyro = ts;
		// Change in angle equals gyro measures * time passed since last measurement
		gyro_angle = gyro_angle * dt_gyro;
		// Apply the calculated change of angle to the current angle (theta)
		std::lock_guard<std::mutex> lock(theta_mtx);
		theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
	}

	void process_accel(rs2_vector accel_data)
	{
		// Holds the angle as calculated from accelerometer data
		float3 accel_angle;
		// Calculate rotation angle from accelerometer data
		accel_angle.z = atan2(accel_data.y, accel_data.z);
		accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));
		// If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
		std::lock_guard<std::mutex> lock(theta_mtx);
		if (first)
		{
			first = false;
			theta = accel_angle;
			// Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
			theta.y = PI;
		}
		else
		{
			/*

			Apply Complementary Filter:

				- high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals

				  that are steady over time, is used to cancel out drift.

				- low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations

			*/
			theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
			theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
		}
	}

	// Returns the current rotation angle

	float3 get_theta()
	{
		std::lock_guard<std::mutex> lock(theta_mtx);
		return theta;
	}
};

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

	size_t gryro_iter = 0;
	size_t accel_iter = 0;
	size_t depth_iter = 0;
	size_t color_iter = 0;
	double ts_gyro = 0.0;
	double ts_accel = 0.0;
	double ts_depth = 0.0;
	double ts_color = 0.0;
	double depth_gryo_latency = 0.0;
	rs2_timestamp_domain gryo_domain;
	rs2_timestamp_domain accel_domain;
	rs2_timestamp_domain depth_domain;
	rs2_timestamp_domain color_domain;

	rs2::config config;
	rs2::device device;
	rs2::pipeline pipe;

	// enable file playback with playback repeat disabled
	config.enable_device_from_file(file_name, false);

	// start pipeline and get device
	// pipeline_profile = pipe.start( config );


	rs2::frameset frames;

	size_t frame_index = 0;

	bool stop = false;
	bool playing = false;

	auto pipeline_profile = pipe.start(config, [&](rs2::frame frame) {
		playing = true;
		try
		{
			if (rs2::motion_frame motion = frame.as<rs2::motion_frame>())
			{
				if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL)
				{
					std::cout<< std::setprecision(0) << std::fixed << "Got Accel. TS: " << motion.get_timestamp() << std::endl;
				}
				else if (motion.get_profile().stream_type() == RS2_STREAM_GYRO)
				{
					std::cout<< std::setprecision(0) << std::fixed << "Got Gyro. TS: " << motion.get_timestamp() << std::endl;
				}
				
			}
			else if (frame.get_profile().stream_type() == RS2_STREAM_COLOR)
			{
				std::cout<< std::setprecision(0) << std::fixed << "Got Color. TS: " << frame.get_timestamp() << std::endl;
			}
			else if (frame.get_profile().stream_type() == RS2_STREAM_DEPTH)
			{
				std::cout<< std::setprecision(0) << std::fixed << "Got Depth. TS: " << frame.get_timestamp() << std::endl;
			}
			else if (frame.get_profile().stream_type() == RS2_STREAM_INFRARED)
			{
				std::cout<< std::setprecision(0) << std::fixed << "Got Infrared. TS: " << frame.get_timestamp() << std::endl;
			}

			// auto fs = frames.get_color_frame();
			// Add some frames processing here...

			std::cout << "successfully retrieved frame #" << ++frame_index << " (" << ")" <<std::endl;
			/* code */
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
		}
		});

	// get playback device and disable realtime mode
	// device = pipeline_profile.get_device();
	// auto playback = device.as<rs2::playback>();
	// playback.set_real_time( false );

	while (!stop)
	{
		using namespace std::chrono_literals;
		// stop = playing && ( playback.current_status() == RS2_PLAYBACK_STATUS_STOPPED );
		// std::cout <<"Stop: "<<  stop <<std::endl;
		std::this_thread::sleep_for( 10ms );
	}



	std::cout << std::endl << "successfully ended file playback" << std::endl;

	pipe.stop();

	return EXIT_SUCCESS;
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
	rs2_timestamp_domain gryo_domain;
	rs2_timestamp_domain accel_domain;
	rs2_timestamp_domain depth_domain;
	rs2_timestamp_domain color_domain;

	rs2::log_to_console(rs2_log_severity::RS2_LOG_SEVERITY_WARN);
	
	// Create pipeline for gyro and accelerometer. Run in seperate thread.
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe_motion;
	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg_motion;
	// Add streams of gyro and accelerometer to configuration
	cfg_motion.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
	cfg_motion.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
	// Declare object that handles camera pose calculations
	rotation_estimator algo;
	double my_clock = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();

	// Start streaming with the given configuration;
	// Note that since we only allow IMU streams, only single frames are produced
	auto profile = pipe_motion.start(cfg_motion, [&](rs2::frame frame) {
		// Cast the frame that arrived to motion frame
		auto motion = frame.as<rs2::motion_frame>();
		// If casting succeeded and the arrived frame is from gyro stream
		if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
		{
			// Get the timestamp of the current frame
			ts_gyro = motion.get_timestamp();
			gryo_domain = motion.get_frame_timestamp_domain();
			// Get gyro measures
			rs2_vector gyro_data = motion.get_motion_data();
			// Call function that computes the angle of motion based on the retrieved measures
			algo.process_gyro(gyro_data, ts_gyro);
			gryro_iter++;
		}
		// If casting succeeded and the arrived frame is from accelerometer stream
		if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
		{
			// Get accelerometer measures
			ts_accel = motion.get_timestamp();
			accel_domain = motion.get_frame_timestamp_domain();
			rs2_vector accel_data = motion.get_motion_data();
			algo.process_accel(accel_data);
			accel_iter++;
		}
		});
	// Create pipeline for depth and color. Run in this main thread.
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe_camera;
	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg_camera;
	// Add streams of gyro and accelerometer to configuration
	cfg_camera.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg_camera.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	rs2::pipeline_profile profile_camera;


	profile_camera = pipe_camera.start(cfg_camera);

	// const std::vector<rs2::sensor> &vector = profile_camera.get_device().query_sensors();
	// for (const auto & i : vector) {
	// 	if (i.is<rs2::roi_sensor>()){
	// 		i.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0);
	// 	}
	// }

	while (true)
	{
		rs2::frameset data = pipe_camera.wait_for_frames();
		rs2::frame depth = data.get_depth_frame();
		rs2::frame color = data.get_color_frame();

		depth_iter++;
		color_iter++;

		ts_depth = depth.get_timestamp();
		ts_color = color.get_timestamp();

		depth_domain = depth.get_frame_timestamp_domain();
		color_domain = color.get_frame_timestamp_domain();

		depth_gryo_latency = ts_gyro - ts_depth;

		double new_clock = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
		if (new_clock - my_clock >= 1000)

		{

			my_clock = new_clock;

			std::cout << std::setprecision(0) << std::fixed << "FPS --- "
				<< "Gryo: " << gryro_iter << "; Accel: " << accel_iter

				<< "; Depth: " << depth_iter << "; RGB: " << color_iter << std::endl;

			std::cout << std::setprecision(0) << std::fixed << "Timing --- Now: " << my_clock << "; Gryo: " << ts_gyro << "; Accel: " << ts_accel

				<< "; Depth: " << ts_depth << "; RGB: " << ts_color << std::endl;

			std::cout << std::setprecision(0) << std::fixed << "Time Domain --- Now: " << my_clock << "; Gryo: " << gryo_domain << "; Accel: " << accel_domain

				<< "; Depth: " << depth_domain << "; RGB: " << color_domain << std::endl;

			std::cout << std::setprecision(0) << std::fixed << "Latency --- DepthToGryo: " << depth_gryo_latency << std::endl;

			std::cout << std::endl;

			gryro_iter = 0;
			accel_iter = 0;
			depth_iter = 0;
			color_iter = 0;
		}
		// auto domain = depth.get_frame_timestamp_domain();
	}
	// Stop the pipeline
	pipe_motion.stop();
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
