#include "utility.hpp"

namespace rstracker
{

IMUHistory::IMUHistory(size_t size)
{
    m_max_size = size;
}

void IMUHistory::add_data(sensor_name module, IMUData data)
{
    m_map[module].push_front(data);
    if (m_map[module].size() > m_max_size)
        m_map[module].pop_back();
}
bool IMUHistory::is_data_full(sensor_name module)
{
    return m_map[module].size() == m_max_size;
}

const std::list<IMUData> &IMUHistory::get_data(sensor_name module)
{
    return m_map[module];
}
IMUData IMUHistory::recent_data(sensor_name module)
{
    return m_map[module].front();
}
IMUData IMUHistory::old_data(sensor_name module)
{
    auto old = std::next(m_map[module].begin(), 1);
    return *old;
}

IMUData IMUHistory::really_old_data(sensor_name module)
{
    return m_map[module].back();
}

std::string float3_to_string(Float3 v)
{
    std::stringstream ss;
    ss << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return ss.str();
}

std::string vec3_to_string(rstracker_pb::Vec3 v)
{
    std::stringstream ss;
    ss << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    return ss.str();
}

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

void print_profiles(std::vector<StreamDetail> stream_details)
{

    std::cout << std::setw(22) << "    device"  << std::setw(16) << "    stream" << std::setw(16)
            << " resolution" << std::setw(10) << " fps" << std::setw(10) << " format" << std::endl;
	for (auto &stream: stream_details)
	{
        std::string res = std::to_string(stream.width) + "x" + std::to_string(stream.height);
        std::string fps = "@ " + std::to_string(stream.fps) + "Hz";
        std::cout << std::setw(22) << stream.device_name << std::setw(16) << stream.name << std::setw(16) <<  res 
        << std::setw(10) << fps<< std::setw(10) << stream.format << std::endl;

	}
}

void print_profiles(std::vector<rs2::stream_profile> streams)
{


    std::cout << "Supported modes:\n" << std::setw(16) << "    stream" << std::setw(16)
            << " resolution" << std::setw(10) << " fps" << std::setw(10) << " format" << std::endl;
	for (auto &stream: streams)
	{
        if (auto video = stream.as<rs2::video_stream_profile>())
        {
            std::cout << "    " << stream.stream_name() << "\t\t  " << video.width() << "x"
                << video.height() << "\t@ " << stream.fps() << std::setw(6) << "Hz\t" << stream.format() << std::endl;
        }
        else
        {
            std::cout << "    " << stream.stream_name() << "\t N/A\t\t@ " << stream.fps()
                << std::setw(6) << "Hz\t" << stream.format() << std::endl;
        }
	}
}

StreamDetail stream_profile_to_details(rs2::stream_profile sp)
{
    int w=0,h=0;
    if (auto video = sp.as<rs2::video_stream_profile>())
    {
        w = video.width();
        h = video.height();
    }
    StreamDetail sd_ = {"", sp.stream_name(), STRM_ENUM.at(sp.stream_name()), w, h, sp.fps(), sp.format()};
    return sd_;
}


void print_message(rstracker_pb::IMUMessage &msg, IMUHistory &imu_hist)
{
    // values for controlling format
    const int time_width = 15 ;
    const int sensor_width = 40;
    const int num_flds = 4;
    const std::string sep = " |" ;
    const int total_width = time_width*2 + sensor_width*2 + sep.size() * num_flds ;
    const std::string line = sep + std::string( total_width-1, '-' ) + '|' ;

    double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::cout<< std::setprecision(0) << std::fixed << "Current ts: " << now << std::endl;
    std::cout << line << '\n' << sep
              << std::setw(time_width) <<"timestamp" << sep
              << std::setw(sensor_width) << "accel" << sep 
              << std::setw(time_width) << "timestamp" << sep
              << std::setw(sensor_width) << "gryo" << sep << '\n' << line << '\n' ;

    std::cout << sep << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.really_old_data(mACCEL).ts << sep
              << std::setw(sensor_width) << float3_to_string(imu_hist.really_old_data(mACCEL).data) << sep 
              << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.really_old_data(mGYRO).ts << sep
              << std::setw(sensor_width) << float3_to_string(imu_hist.really_old_data(mGYRO).data) << sep <<  '\n';
    
    std::cout << sep << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.old_data(mACCEL).ts << sep
              << std::setw(sensor_width) << float3_to_string(imu_hist.old_data(mACCEL).data) << sep 
              << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.old_data(mGYRO).ts << sep
              << std::setw(sensor_width) << float3_to_string(imu_hist.old_data(mGYRO).data) << sep <<  '\n';

    std::cout << sep << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.recent_data(mACCEL).ts << sep
              << std::setw(sensor_width) << float3_to_string(imu_hist.recent_data(mACCEL).data) << sep 
              << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.recent_data(mGYRO).ts << sep
              << std::setw(sensor_width) << float3_to_string(imu_hist.recent_data(mGYRO).data) << sep << "\n";
    // Interplated
    std::cout << sep << std::setw(time_width) << std::setprecision(0) << std::fixed << msg.hardware_ts() << sep
              << std::setw(sensor_width) << vec3_to_string(msg.accel()) << sep 
              << std::setw(time_width) << std::setprecision(0) << std::fixed << msg.hardware_ts()  << sep
              << std::setw(sensor_width) << vec3_to_string(msg.gyro()) << sep << "\n";
    

    std::cout << line << '\n' ;

    std::cout << std::endl;
}


} // namespace rstracker
