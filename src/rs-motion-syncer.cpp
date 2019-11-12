// License: Apache 2.0. See LICENSE file in root directory.

// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>

#include <mutex>

#include "example.hpp" // Include short list of convenience functions for rendering

#include <cstring>

#include <chrono>
#include <thread>
#include "pipeline/pipeline.h"
#include "pipeline/resolver.h"

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

struct stream_profile_wrapper
{
    rs2_stream stream_type;
    int width;
    int height;
    int fps;
    rs2_format format;
    rs2::sensor sensor;
    rs2::stream_profile stream_profile;
};

void get_sensor_and_profiles(rs2::device &dev, std::vector<stream_profile_wrapper> &my_profiles)
{
    auto sensors = dev.query_sensors();
    // rs2::stream_profile color_profile = {RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_BGR8, 30};
    for (auto &&sensor : dev.query_sensors())
    {
        std::cout << "Stream Profiles supported by " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;

        std::cout << " Supported modes:\n"
                  << std::setw(16) << "    stream" << std::setw(16)
                  << " resolution" << std::setw(10) << " fps" << std::setw(10) << " format" << std::endl;
        // Show which streams are supported by this device
        for (auto &&profile : sensor.get_stream_profiles())
        {
            // profile.stream_type()
            if (auto video = profile.as<rs2::video_stream_profile>())
            {

                std::cout << video.stream_index() << "    " << profile.stream_name() << "\t  " << video.width() << "x"
                          << video.height() << "\t@ " << profile.fps() << std::setw(6) << "Hz\t" << profile.format() << std::endl;

                for (auto &&my_profile : my_profiles)
                {
                    auto same_type = my_profile.stream_type == profile.stream_type();
                    auto same_fps = my_profile.fps == profile.fps();
                    auto same_width = my_profile.width == video.width();
                    auto same_height = my_profile.height == video.height();
                    auto same_format = my_profile.format == video.format();
                    if (same_type && same_fps && same_width && same_height && same_format)
                    {
                        my_profile.sensor = sensor;
                        my_profile.stream_profile = profile;
                        std::cout << "FOUND IT!!!" << profile.stream_name() << std::endl;
                    }
                }
            }
            else
            {
                std::cout << profile.stream_index() << "    " << profile.stream_name() << "\t N/A\t\t@ " << profile.fps()
                          << std::setw(6) << "Hz\t" << profile.format() << std::endl;
                                for (auto &&my_profile : my_profiles)
                {
                    auto same_type = my_profile.stream_type == profile.stream_type();
                    auto same_fps = my_profile.fps == profile.fps();
                    auto same_format = my_profile.format == video.format();
                    if (same_type && same_fps && same_format)
                    {
                        my_profile.sensor = sensor;
                        my_profile.stream_profile = profile;
                        std::cout << "FOUND IT!!!" << profile.stream_name() << std::endl;
                    }
                }
            }
        }

        std::cout << std::endl;
    }
}

int main(int argc, char *argv[]) try

{
    using namespace std::chrono_literals;

    // Before running the example, check that a device supporting IMU is connected

    if (!check_imu_is_supported())

    {

        std::cerr << "Device supporting IMU (D435i) not found";

        return EXIT_FAILURE;
    }

    // Initialize window for rendering

    // window app(1280, 720, "RealSense Motion Example");

    // // Construct an object to manage view state

    // glfw_state app_state(0.0, 0.0);

    // // Register callbacks to allow manipulation of the view state

    // register_glfw_callbacks(app, app_state);

    size_t gryro_iter = 0;

    size_t accel_iter = 0;

    size_t depth_iter = 0;

    size_t color_iter = 0;

    double ts_gyro = 0.0;

    double ts_accel = 0.0;

    double ts_depth = 0.0;

    double ts_color = 0.0;

    // Create pipeline for depth and color. Run in this main thread.

    // Declare RealSense pipeline, encapsulating the actual device and sensors

    // rs2::pipeline pipe_camera;

    // Create a configuration for configuring the pipeline with a non default profile

    // Add streams of gyro and accelerometer to configuration
    double my_clock = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();

    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::device dev = list.front();


    stream_profile_wrapper depth_profile({RS2_STREAM_DEPTH, 640, 480, 30, RS2_FORMAT_Z16});
    stream_profile_wrapper gyro_profile({RS2_STREAM_GYRO, 0, 0, 200, RS2_FORMAT_MOTION_XYZ32F});

    std::vector<stream_profile_wrapper> my_profiles = {depth_profile, gyro_profile};
    get_sensor_and_profiles(dev, my_profiles);

    depth_profile = my_profiles.front();
    depth_profile.sensor.open(depth_profile.stream_profile);

    gyro_profile = my_profiles.front();
    gyro_profile.sensor.open(gyro_profile.stream_profile);

    //Creating frame queue and setting the queue capacity
    #define CAPACITY 4
    rs2::frame_queue fq_camera(CAPACITY);
    depth_profile.sensor.start(fq_camera);
    rs2::frame_queue fq_motion(CAPACITY);
    gyro_profile.sensor.start(fq_motion);
    
    while(true)
    {
        try
        {
            rs2::frame f = fq_camera.wait_for_frame(1);
            ts_depth = f.get_timestamp();   
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            continue;
        }

        try
        {
            rs2::frame f = fq_motion.wait_for_frame(1);
            ts_gyro = f.get_timestamp();   
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            continue;
        }
        

        double new_clock = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
        my_clock = new_clock;
        std::cout << std::setprecision(0) << std::fixed << "FPS --- "
                    << "Gryo: " << gryro_iter << "; Accel: " << accel_iter

                    << "; Depth: " << depth_iter << "; RGB: " << color_iter << std::endl;

        std::cout << std::setprecision(0) << std::fixed << "Timing --- Now: " << my_clock << "; Gryo: " << ts_gyro << "; Accel: " << ts_accel

                    << "; Depth: " << ts_depth << "; RGB: " << ts_color << std::endl;

    }

    // librealsense::util::config cfg_camera;

    // cfg_camera.enable_stream(RS2_STREAM_COLOR, -1, 640, 480, RS2_FORMAT_BGR8, 30);
    // cfg_camera.enable_stream(RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 30);
    // cfg_camera.enable_stream(RS2_STREAM_ACCEL, -1, 0, 0, RS2_FORMAT_MOTION_XYZ32F, 0);
    // cfg_camera.enable_stream(RS2_STREAM_GYRO, -1, 0, 0, RS2_FORMAT_MOTION_XYZ32F, 0);

    // auto pipeline_profile = cfg_camera.resolve(dev);

    // pipeline_profile.get_streams();
    // pipeline_profile.

    // rs2::device selected_device = profile_camera.get_device();
    // auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    // auto color_sensor = selected_device.first<rs2::color_sensor>();

    // // Main loop

    // // while (app)
    // while (true)

    // {

    //     rs2::frameset data = pipe_camera.wait_for_frames();

    //     rs2::frame depth = data.get_depth_frame();

    //     rs2::frame color = data.get_color_frame();

    //     depth_iter++;

    //     color_iter++;

    //     ts_depth = depth.get_timestamp();

    //     ts_color = color.get_timestamp();

    //     double new_clock = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();

    //     std::cout << new_clock << std::endl;

    //     if (new_clock - my_clock >= 1000)

    //     {

    //         my_clock = new_clock;

    //         std::cout << std::setprecision(0) << std::fixed << "FPS --- "
    //                   << "Gryo: " << gryro_iter << "; Accel: " << accel_iter

    //                   << "; Depth: " << depth_iter << "; RGB: " << color_iter << std::endl;

    //         std::cout << std::setprecision(0) << std::fixed << "Timing --- Now: " << my_clock << "; Gryo: " << ts_gyro << "; Accel: " << ts_accel

    //                   << "; Depth: " << ts_depth << "; RGB: " << ts_color << std::endl;

    //         std::cout << std::endl;

    //         gryro_iter = 0;

    //         accel_iter = 0;

    //         depth_iter = 0;

    //         color_iter = 0;
    //     }

    //     // auto domain = depth.get_frame_timestamp_domain();

    //     // Configure scene, draw floor, handle manipultation by the user etc.

    //     // render_scene(app_state);

    //     // Draw the camera according to the computed theta

    //     // camera.render_camera(algo.get_theta());
    //     std::this_thread::sleep_for(10ms);
    // }

    // // Stop the pipeline

    // return EXIT_SUCCESS;
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
