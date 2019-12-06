#ifndef RSTRACKER_UTILITY_H
#define RSTRACKER_UTILITY_H

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
#include <librealsense2/rs.hpp>

#include "IMUMessage.pb.h"

#define MS_TO_NS 1000000

namespace rstracker
{

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

void print_message(rstracker_pb::IMUMessage &msg, IMUHistory &imu_hist);

void print_profiles(std::vector<rs2::stream_profile> streams);

}

#endif // RSTRACKER_UTILITY_H