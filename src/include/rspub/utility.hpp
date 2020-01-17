#ifndef RSTRACKER_UTILITY_H
#define RSTRACKER_UTILITY_H

#include <cstring>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <math.h>
#include <string>
#include <sstream>
#include <list>
#include <iostream>
#include <algorithm>
#include <iomanip>

#include <librealsense2/rs.hpp>
#include <glog/logging.h>
#include <toml.hpp>

#include "PoseMessage.pb.h"
#include "ImageMessage.pb.h"
#include "IMUMessage.pb.h"
#include "PointCloudMessage.pb.h"

#include "rspub/types_simple.hpp"

#define MS_TO_NS 1000000

namespace rspub
{

// quaternion inner product squared
double norm_l2(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2);
double quat_product(const double &x1, const double &y1, const double &z1, const double &w1, const double &x2, const double &y2, const double &z2, const double &w2);
double quat_angle_diff(rspub_pb::Vec4 &quat1, rspub_pb::Vec4 &quat2);

std::string pad_int(int num, int pad=8);

class Float3
{
    public:
        float x, y, z;

    public:
        Float3& operator*=(const float& factor)
        {
            x*=factor;
            y*=factor;
            z*=factor;
            return (*this);
        }
        Float3 operator*(const float& factor)
        {
            Float3 data(*this);
            data.x*=factor;
            data.y*=factor;
            data.z*=factor;
            return data;
        }
        Float3& operator+=(const Float3& other)
        {
            x+=other.x;
            y+=other.y;
            z+=other.z;
            return (*this);
        }
        Float3 operator+(const Float3& other)
        {
            Float3 data(*this);
            data.x+=other.x;
            data.y+=other.y;
            data.z+=other.z;
            return data;
        }
};


// class IMUMessage
// {
//     public:
//         IMUMessage():
//             gyro(),
//             accel(),
//             ts()
//             {};
//         IMUMessage(const Float3 gyro, const Float3 accel, double time):
//             gyro(gyro),
//             accel(accel),
//             ts(time)
//             {};
        
//     public:
//         Float3 gyro;
//         Float3 accel;
//         double ts;
// };

class IMUData
{
    public:
        IMUData(const IMUData& other):
            IMUData(other.data, other.ts)
            {};
        IMUData(const Float3 reading, double time):
            data(reading),
            ts(time)
            {};
        IMUData operator*(const double factor);
        IMUData operator+(const IMUData& other);
    public:
        Float3 data;
        double ts;
};
enum 
sensor_name {mGYRO, mACCEL};

class IMUHistory
{

    private:
        size_t m_max_size;
        std::map<sensor_name, std::list<IMUData> > m_map;

    public:
        IMUHistory(size_t size);
        void add_data(sensor_name module, IMUData data);
        bool is_data_full(sensor_name);
        // bool is_data(sensor_name);
        const std::list<IMUData>& get_data(sensor_name module);
        IMUData recent_data(sensor_name module);
        IMUData old_data(sensor_name module);
        IMUData really_old_data(sensor_name module);
};

bool check_imu_is_supported();

class NamedFilter
{
	public:
		std::string _name;
		std::shared_ptr<rs2::filter> _filter;

	public:
		NamedFilter(std::string name, std::shared_ptr<rs2::filter> filter):
		_name(name), _filter(filter)
		{}
};

struct StreamDetail
{
    std::string device_name;
	std::string stream_name;
    rs2_stream stream_type;
	int width;
	int height;
	int fps;
	rs2_format format;

	bool operator==(const StreamDetail& rhs) const
	{
		return stream_name == rhs.stream_name && width == rhs.width && height == rhs.height && fps == rhs.fps && format == rhs.format;
	}
};

void parse_desired_stream(std::vector<StreamDetail> &sds, const toml::value &tcf, std::string streams_str);
bool create_filters(std::vector<NamedFilter> &filters, const toml::value &tcf);

void fill_pose_message(rs2_pose &pose, rspub_pb::PoseMessage &pm, double ts);
void fill_image_message(rs2::video_frame &dframe, rspub_pb::ImageMessage &frame_image);
void fill_image_message_second(rs2::video_frame &dframe, rspub_pb::ImageMessage &frame_image);
void fill_pointcloud(rs2::depth_frame &dframe, rs2::video_frame &cframe, rs2::pointcloud &pc, rs2::points &points, bool color);
void fill_pointcloud_message(rs2::points points, rs2::video_frame &cframe, rspub_pb::PointCloudMessage &pc_message, double hardware_ts, bool color);
void fill_pointcloud_message_optimized(rs2::depth_frame &dframe, rs2::video_frame &cframe,rspub_pb::PointCloudMessage &pc_message,
										 double hardware_ts, int stride=1, bool color=false);

void print_message(rspub_pb::IMUMessage &msg, IMUHistory &imu_hist);
void print_profiles(std::vector<rs2::stream_profile> streams);
void print_profiles(std::vector<StreamDetail> stream_details);
const static std::unordered_map<std::string,rs2_stream> STRM_ENUM{
    {"Any",RS2_STREAM_ANY},
    {"Depth",RS2_STREAM_DEPTH},
    {"Color",RS2_STREAM_COLOR},
    {"Infrared 1",RS2_STREAM_INFRARED},
    {"Infrared 2",RS2_STREAM_INFRARED},
    {"Fisheye 1",RS2_STREAM_FISHEYE},
    {"Fisheye 2",RS2_STREAM_FISHEYE},
    {"Gyro",RS2_STREAM_GYRO},
    {"Accel",RS2_STREAM_ACCEL},
    {"Pose",RS2_STREAM_POSE}
};

const static std::unordered_map<std::string,rs2_format> FMT_ENUM{
    {"Any",RS2_FORMAT_ANY},
    {"Z16",RS2_FORMAT_Z16},
    {"BGR8",RS2_FORMAT_BGR8},
    {"6DOF",RS2_FORMAT_6DOF}
};


StreamDetail stream_profile_to_details(rs2::stream_profile sp);



}

#endif // RSTRACKER_UTILITY_H