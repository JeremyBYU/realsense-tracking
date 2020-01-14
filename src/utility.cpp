#include "rspub/utility.hpp"

#include "PoseMessage.pb.h"
namespace rspub
{

    
void parse_desired_stream(std::vector<StreamDetail> &sds, const toml::value &tcf, std::string streams_str)
{
	try
	{
		auto streams  = toml::find<std::vector<toml::value>>(tcf, streams_str);
		for (auto &stream: streams)
		{
			try
			{
				auto active = toml::find<bool>(stream, "active");
				auto device_name = toml::find<std::string>(stream, "device_name");
				auto stream_name = toml::find<std::string>(stream, "stream_name");
				auto width = toml::find<int>(stream, "width");
				auto height = toml::find<int>(stream, "height");
				auto framerate = toml::find<int>(stream, "framerate");
				auto format = toml::find<std::string>(stream, "format");
				if (active)
					sds.push_back({device_name, stream_name, rspub::STRM_ENUM.at(stream_name), width, height, framerate, rspub::FMT_ENUM.at(format)});
			}
			catch(const std::exception& e){std::cerr << e.what() << '\n';}
		}
	}
	catch(const std::exception& e){std::cerr << e.what() << '\n';}
}

bool create_filters(std::vector<NamedFilter> &filters, const toml::value &tcf)
{
	try
	{
		auto filters_t  = toml::find(tcf, "filters");
		auto disparity = false;
		// Decimation Filter
		try
		{
			auto filter  = toml::find(filters_t, "decimation");
			auto active = toml::find<bool>(filter, "active");
			auto magnitude = toml::find<float>(filter, "magnitude");
			if (active)
				filters.push_back(NamedFilter("decimation", std::make_shared<rs2::decimation_filter>(magnitude)));
		}
		catch(const std::exception& e){std::cerr << e.what() << '\n';}

		// Disparity Filter
		try
		{
			auto filter  = toml::find(filters_t, "disparity");
			auto active = toml::find<bool>(filter, "active");
			disparity = true;
			if (active)
				filters.push_back(NamedFilter("disparity", std::make_shared<rs2::disparity_transform>(true)));
		}
		catch(const std::exception& e){std::cerr << e.what() << '\n';}

		// Temporal Filter
		try
		{
			auto filter  = toml::find(filters_t, "temporal");
			auto active = toml::find<bool>(filter, "active");
			auto smooth_alpha = toml::find<float>(filter, "smooth_alpha");
			auto smooth_delta = toml::find<float>(filter, "smooth_delta");
			auto persistence_control = toml::find<float>(filter, "persistence_control");
			if (active)
				filters.push_back(NamedFilter("temporal", std::make_shared<rs2::temporal_filter>(smooth_alpha, smooth_delta, persistence_control)));
		}
		catch(const std::exception& e){std::cerr << e.what() << '\n';}

		// Spatial Filter
		try
		{
			auto filter  = toml::find(filters_t, "spatial");
			auto active = toml::find<bool>(filter, "active");
			auto smooth_alpha = toml::find<float>(filter, "smooth_alpha");
			auto smooth_delta = toml::find<float>(filter, "smooth_delta");
			auto magnitude = toml::find<float>(filter, "magnitude");
			auto hole_fill = toml::find<float>(filter, "hole_fill");
			if (active)
				filters.push_back(NamedFilter("spatial", std::make_shared<rs2::spatial_filter>(smooth_alpha, smooth_delta, magnitude, hole_fill)));
		}
		catch(const std::exception& e){std::cerr << e.what() << '\n';}

		// Reverse Disparity
		if (disparity)
			filters.push_back(NamedFilter("disparity", std::make_shared<rs2::disparity_transform>(false)));

		// Alignment Filter
		try
		{
			auto filter  = toml::find(filters_t, "align");
			auto active = toml::find<bool>(filter, "active");
			if (active)
				return active;
		}
		catch(const std::exception& e){std::cerr << e.what() << '\n';}
		
		/* code */
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}

	return false;
	
}

void fill_pointcloud(rs2::depth_frame &dframe, rs2::video_frame &cframe, rs2::pointcloud &pc, rs2::points &points, bool color=false)
{
    if (color && cframe)
    {
        pc.map_to(cframe);
    }
    points = pc.calculate(dframe);
}

void fill_pointcloud_message(rs2::points points, rspub_pb::PointCloudMessage &pc_message, double hardware_ts, bool color=false)
{
    auto n_points = points.size();
    auto bpp = 4 * 3; //  4 bytes per float, xyz=3,
    auto n_bytes = n_points * bpp;
    auto vertices = points.get_vertices();
    const char *pc_data = (const char *)(vertices);
    auto format = color ? 0:1;

    // LOG(INFO) << "n_points: " << n_points << "; n_bytes: " << n_bytes;

    double now1 = std::chrono::duration<double, std::micro>(std::chrono::system_clock::now().time_since_epoch()).count();
    pc_message.set_pc_data(pc_data, n_bytes);
    pc_message.set_n_points(n_points);
    pc_message.set_hardware_ts(hardware_ts);
    pc_message.set_bpp(4);
    pc_message.set_format(format);
    double now2 = std::chrono::duration<double, std::micro>(std::chrono::system_clock::now().time_since_epoch()).count();
    // LOG(INFO) << "Copy PC to Protobuff Message: " << now2 -now1;


}

void fill_image_message(rs2::video_frame &dframe, rspub_pb::ImageMessage &frame_image)
{
	const int w = dframe.get_width();
	const int h = dframe.get_height();
	const char *image_data = static_cast<const char *>(dframe.get_data());
	int nbytes = w * h * dframe.get_bytes_per_pixel();
	auto format = dframe.get_profile().format();
	// std::cout << "Depth bpp: " << dframe.get_bytes_per_pixel() << "; Total Bytes: " <<  nbytes << std::endl;

	frame_image.set_hardware_ts(dframe.get_timestamp());
	frame_image.set_width(w);
	frame_image.set_height(h);
	frame_image.set_image_data(image_data, nbytes);
	frame_image.set_bpp(dframe.get_bytes_per_pixel());
	frame_image.set_format(format);
}

void fill_pose_message(rs2_pose &pose, rspub_pb::PoseMessage &pm, double ts)
{
	auto tr = pm.mutable_translation();
	tr->set_x(pose.translation.x);
	tr->set_y(pose.translation.y);
	tr->set_z(pose.translation.z);

	auto vl = pm.mutable_velocity();
	vl->set_x(pose.velocity.x);
	vl->set_y(pose.velocity.y);
	vl->set_z(pose.velocity.z);

	auto ac = pm.mutable_acceleration();
	ac->set_x(pose.acceleration.x);
	ac->set_y(pose.acceleration.y);
	ac->set_z(pose.acceleration.z);

	auto rot = pm.mutable_rotation();
	rot->set_x(pose.rotation.x);
	rot->set_y(pose.rotation.y);
	rot->set_z(pose.rotation.z);
	rot->set_w(pose.rotation.w);

	auto avl = pm.mutable_angular_velocity();
	avl->set_x(pose.angular_velocity.x);
	avl->set_y(pose.angular_velocity.y);
	avl->set_z(pose.angular_velocity.z);

	auto aac = pm.mutable_angular_acceleration();
	aac->set_x(pose.angular_acceleration.x);
	aac->set_y(pose.angular_acceleration.y);
	aac->set_z(pose.angular_acceleration.z);

	pm.set_tracker_confidence(pose.tracker_confidence);
	pm.set_mapper_confidence(pose.mapper_confidence);
	pm.set_hardware_ts(ts);
}

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

std::string vec3_to_string(rspub_pb::Vec3 v)
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
        std::cout << std::setw(22) << stream.device_name << std::setw(16) << stream.stream_name << std::setw(16) <<  res 
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


void print_message(rspub_pb::IMUMessage &msg, IMUHistory &imu_hist)
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
