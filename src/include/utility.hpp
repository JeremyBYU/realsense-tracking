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

namespace rstracker
{

class float3
{
    public:
        float x, y, z;

    public:
        float3& operator*=(const float& factor)
        {
            x*=factor;
            y*=factor;
            z*=factor;
            return (*this);
        }
        float3 operator*(const float& factor)
        {
            float3 data(*this);
            data.x*=factor;
            data.y*=factor;
            data.z*=factor;
            return data;
        }
        float3& operator+=(const float3& other)
        {
            x+=other.x;
            y+=other.y;
            z+=other.z;
            return (*this);
        }
        float3 operator+(const float3& other)
        {
            float3 data(*this);
            data.x+=other.x;
            data.y+=other.y;
            data.z+=other.z;
            return data;
        }
};


class IMUMessage
{
    public:
        IMUMessage():
            gyro(),
            accel(),
            ts()
            {};
        IMUMessage(const float3 gyro, const float3 accel, double time):
            gyro(gyro),
            accel(accel),
            ts(time)
            {};
        
    public:
        float3 gyro;
        float3 accel;
        double ts;
};

class IMUData
{
    public:
        IMUData(const IMUData& other):
            IMUData(other.data, other.ts)
            {};
        IMUData(const float3 reading, double time):
            data(reading),
            ts(time)
            {};
        IMUData operator*(const double factor);
        IMUData operator+(const IMUData& other);
    public:
        float3 data;
        double ts;
};
enum sensor_name {mGYRO, mACCEL};

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
};

bool check_imu_is_supported();

void print_message(IMUMessage msg, IMUHistory imu_hist);

}



#endif // RSTRACKER_UTILITY_H