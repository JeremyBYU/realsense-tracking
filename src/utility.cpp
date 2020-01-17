#include "rspub/utility.hpp"

#include "PoseMessage.pb.h"
#define DEPTH_METER 0.001
namespace rspub
{

// quaternion inner product squared
double quat_product(const double &x1, const double &y1, const double &z1, const double &w1, const double &x2, const double &y2, const double &z2, const double &w2)
{
	double inner_product = x1 * x2 + y1 * y2 + z1 * z2 + w1 * w2;
	double inner_product_2 = inner_product * inner_product;
	return inner_product_2;
}

// Approximate radian difference between two quaternions
// https://math.stackexchange.com/questions/90081/quaternion-distance
// NOTE Must be unit quaternions
double quat_angle_diff(rspub_pb::Vec4 &quat1, rspub_pb::Vec4 &quat2)
{
	double inner_product_2 = quat_product(quat1.x(), quat1.y(), quat1.z(), quat1.w(), quat2.x(), quat2.y(), quat2.z(), quat2.w());
	return 1.0 - inner_product_2;
}

double norm_l2(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2)
{
	double dx = x2 - x1;
	double dy = y2 - y1;
	double dz = z2 - z1;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string pad_int(int num, int pad)
{
	std::ostringstream out;
	out << std::internal << std::setfill('0') << std::setw(pad) << num;
	return out.str();
}

void parse_desired_stream(std::vector<StreamDetail> &sds, const toml::value &tcf, std::string streams_str)
{
	try
	{
		auto streams = toml::find<std::vector<toml::value>>(tcf, streams_str);
		for (auto &stream : streams)
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
			catch (const std::exception &e)
			{
				std::cerr << e.what() << '\n';
			}
		}
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << '\n';
	}
}

bool create_filters(std::vector<NamedFilter> &filters, const toml::value &tcf)
{
	try
	{
		auto filters_t = toml::find(tcf, "filters");
		auto disparity = false;
		// Decimation Filter
		try
		{
			auto filter = toml::find(filters_t, "decimation");
			auto active = toml::find<bool>(filter, "active");
			auto magnitude = toml::find<float>(filter, "magnitude");
			if (active)
				filters.push_back(NamedFilter("decimation", std::make_shared<rs2::decimation_filter>(magnitude)));
		}
		catch (const std::exception &e)
		{
			std::cerr << e.what() << '\n';
		}

		// Disparity Filter
		try
		{
			auto filter = toml::find(filters_t, "disparity");
			auto active = toml::find<bool>(filter, "active");
			disparity = true;
			if (active)
				filters.push_back(NamedFilter("disparity", std::make_shared<rs2::disparity_transform>(true)));
		}
		catch (const std::exception &e)
		{
			std::cerr << e.what() << '\n';
		}

		// Temporal Filter
		try
		{
			auto filter = toml::find(filters_t, "temporal");
			auto active = toml::find<bool>(filter, "active");
			auto smooth_alpha = toml::find<float>(filter, "smooth_alpha");
			auto smooth_delta = toml::find<float>(filter, "smooth_delta");
			auto persistence_control = toml::find<float>(filter, "persistence_control");
			if (active)
				filters.push_back(NamedFilter("temporal", std::make_shared<rs2::temporal_filter>(smooth_alpha, smooth_delta, persistence_control)));
		}
		catch (const std::exception &e)
		{
			std::cerr << e.what() << '\n';
		}

		// Spatial Filter
		try
		{
			auto filter = toml::find(filters_t, "spatial");
			auto active = toml::find<bool>(filter, "active");
			auto smooth_alpha = toml::find<float>(filter, "smooth_alpha");
			auto smooth_delta = toml::find<float>(filter, "smooth_delta");
			auto magnitude = toml::find<float>(filter, "magnitude");
			auto hole_fill = toml::find<float>(filter, "hole_fill");
			if (active)
				filters.push_back(NamedFilter("spatial", std::make_shared<rs2::spatial_filter>(smooth_alpha, smooth_delta, magnitude, hole_fill)));
		}
		catch (const std::exception &e)
		{
			std::cerr << e.what() << '\n';
		}

		// Reverse Disparity
		if (disparity)
			filters.push_back(NamedFilter("disparity", std::make_shared<rs2::disparity_transform>(false)));

		// Alignment Filter
		try
		{
			auto filter = toml::find(filters_t, "align");
			auto active = toml::find<bool>(filter, "active");
			if (active)
				return active;
		}
		catch (const std::exception &e)
		{
			std::cerr << e.what() << '\n';
		}

		/* code */
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << '\n';
	}

	return false;
}

void fill_pointcloud(rs2::depth_frame &dframe, rs2::video_frame &cframe, rs2::pointcloud &pc, rs2::points &points, bool color = false)
{
	if (color && cframe)
	{
		pc.map_to(cframe);
	}
	points = pc.calculate(dframe);
}

void deproject_pixel(float *point, const float u, const float v, const float &depth, const rs2_intrinsics &intrin)
{
	float x = (u - intrin.ppx) / intrin.fx;
	float y = (v - intrin.ppy) / intrin.fy;
	point[0] = depth * x;
	point[1] = depth * y;
	point[2] = depth;
}

int create_pointcloud_optimized(float *buffer_pc, uint8_t *buffer_color, rs2::depth_frame &dframe,
								rs2::video_frame &cframe, int stride = 1, const bool color = false)
{
	const uint8_t *color_data = reinterpret_cast<const uint8_t *>(cframe.get_data());
	const uint16_t *depth_data = reinterpret_cast<const uint16_t *>(dframe.get_data());
	auto video = dframe.get_profile().as<rs2::video_stream_profile>();
	auto intrin = video.get_intrinsics();

	const int cl_bp = cframe.get_bytes_per_pixel();
	const int cl_sb = cframe.get_stride_in_bytes();
	int n_points = 0;
	for (int y = 0; y < intrin.height; y += stride)
	{
		for (int x = 0; x < intrin.width; x += stride)
		{
			int index = y * intrin.width + x;
			float depth_value = DEPTH_METER * depth_data[index];
			deproject_pixel(buffer_pc, (float)x, (float)y, depth_value, intrin);
			buffer_pc += 3;
			if (color)
			{
				int index_ = y * cl_sb + x * cl_bp;
				buffer_color[2] = color_data[index_];
				buffer_color[1] = color_data[index_ + 1];
				buffer_color[0] = color_data[index_ + 2];
				buffer_color += 3;
			}
			n_points++;
		}
	}
	return n_points;
}

void fill_pointcloud_message_optimized(rs2::depth_frame &dframe, rs2::video_frame &cframe, rspub_pb::PointCloudMessage &pc_message,
									   double hardware_ts, int stride, const bool color)
{

	auto buffer_pc = pc_message.mutable_pc_data();
	auto buffer_color = pc_message.mutable_color_data();
	auto profile = dframe.get_profile();
	rs2::video_stream_profile video = profile.as<rs2::video_stream_profile>();
	auto intrin = video.get_intrinsics();

	auto bpp_pc = 4 * 3; //  4 bytes per float, xyz=3,
	auto bpp_cl = 1 * 3; //  1 bytes per uint8_t, rgb=3

	// Determine maximum buffer size needed
	int max_points = (intrin.height * intrin.width) / (stride * stride);
	buffer_pc->resize(max_points * bpp_pc);
	if (color)
		buffer_color->resize(max_points * bpp_cl);

	auto pc_ptr = reinterpret_cast<float *>(buffer_pc->data());
	auto color_ptr = reinterpret_cast<uint8_t *>(buffer_color->data());
	auto n_points = create_pointcloud_optimized(pc_ptr, color_ptr, dframe, cframe, stride, color);

	// VLOG(3) << "NPoints: " << n_points << "; Color: " << color ;
	buffer_pc->resize(n_points * bpp_pc);
	if (color)
		buffer_color->resize(n_points * bpp_cl);

	auto n_bytes_pc = n_points * bpp_pc;
	auto n_bytes_cl = n_points * bpp_cl;
	auto format = color ? 1 : 0;

	double now1 = std::chrono::duration<double, std::micro>(std::chrono::system_clock::now().time_since_epoch()).count();
	pc_message.set_n_points(n_points);
	pc_message.set_hardware_ts(hardware_ts);
	pc_message.set_bpp(bpp_pc);
	pc_message.set_format(format);
	double now2 = std::chrono::duration<double, std::micro>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void fill_pointcloud_message(rs2::points points, rs2::video_frame &cframe, rspub_pb::PointCloudMessage &pc_message, double hardware_ts, bool color = false)
{
	auto n_points = points.size();
	auto bpp = 4 * 3; //  4 bytes per float, xyz=3,
	auto n_bytes = n_points * bpp;
	auto vertices = points.get_vertices();
	const char *pc_data = (const char *)(vertices);
	auto format = color ? 0 : 1;

	double now1 = std::chrono::duration<double, std::micro>(std::chrono::system_clock::now().time_since_epoch()).count();
	pc_message.set_pc_data(pc_data, n_bytes);
	pc_message.set_n_points(n_points);
	pc_message.set_hardware_ts(hardware_ts);
	pc_message.set_bpp(4);
	pc_message.set_format(format);
	double now2 = std::chrono::duration<double, std::micro>(std::chrono::system_clock::now().time_since_epoch()).count();

	// if (color)
	// {

	// }

	// LOG(INFO) << "n_points: " << n_points << "; n_bytes: " << n_bytes;

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

void fill_image_message_second(rs2::video_frame &dframe, rspub_pb::ImageMessage &frame_image)
{
	const int w = dframe.get_width();
	const int h = dframe.get_height();
	const char *image_data = static_cast<const char *>(dframe.get_data());
	int nbytes = w * h * dframe.get_bytes_per_pixel();
	auto format = dframe.get_profile().format();

	frame_image.set_image_data_second(image_data, nbytes);
	frame_image.set_bpp_second(dframe.get_bytes_per_pixel());
	frame_image.set_format_second(format);
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

	std::cout << std::setw(22) << "    device" << std::setw(16) << "    stream" << std::setw(16)
			  << " resolution" << std::setw(10) << " fps" << std::setw(10) << " format" << std::endl;
	for (auto &stream : stream_details)
	{
		std::string res = std::to_string(stream.width) + "x" + std::to_string(stream.height);
		std::string fps = "@ " + std::to_string(stream.fps) + "Hz";
		std::cout << std::setw(22) << stream.device_name << std::setw(16) << stream.stream_name << std::setw(16) << res
				  << std::setw(10) << fps << std::setw(10) << stream.format << std::endl;
	}
}

void print_profiles(std::vector<rs2::stream_profile> streams)
{

	std::cout << "Supported modes:\n"
			  << std::setw(16) << "    stream" << std::setw(16)
			  << " resolution" << std::setw(10) << " fps" << std::setw(10) << " format" << std::endl;
	for (auto &stream : streams)
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
	int w = 0, h = 0;
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
	const int time_width = 15;
	const int sensor_width = 40;
	const int num_flds = 4;
	const std::string sep = " |";
	const int total_width = time_width * 2 + sensor_width * 2 + sep.size() * num_flds;
	const std::string line = sep + std::string(total_width - 1, '-') + '|';

	double now = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
	std::cout << std::setprecision(0) << std::fixed << "Current ts: " << now << std::endl;
	std::cout << line << '\n'
			  << sep
			  << std::setw(time_width) << "timestamp" << sep
			  << std::setw(sensor_width) << "accel" << sep
			  << std::setw(time_width) << "timestamp" << sep
			  << std::setw(sensor_width) << "gryo" << sep << '\n'
			  << line << '\n';

	std::cout << sep << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.really_old_data(mACCEL).ts << sep
			  << std::setw(sensor_width) << float3_to_string(imu_hist.really_old_data(mACCEL).data) << sep
			  << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.really_old_data(mGYRO).ts << sep
			  << std::setw(sensor_width) << float3_to_string(imu_hist.really_old_data(mGYRO).data) << sep << '\n';

	std::cout << sep << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.old_data(mACCEL).ts << sep
			  << std::setw(sensor_width) << float3_to_string(imu_hist.old_data(mACCEL).data) << sep
			  << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.old_data(mGYRO).ts << sep
			  << std::setw(sensor_width) << float3_to_string(imu_hist.old_data(mGYRO).data) << sep << '\n';

	std::cout << sep << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.recent_data(mACCEL).ts << sep
			  << std::setw(sensor_width) << float3_to_string(imu_hist.recent_data(mACCEL).data) << sep
			  << std::setw(time_width) << std::setprecision(0) << std::fixed << imu_hist.recent_data(mGYRO).ts << sep
			  << std::setw(sensor_width) << float3_to_string(imu_hist.recent_data(mGYRO).data) << sep << "\n";
	// Interplated
	std::cout << sep << std::setw(time_width) << std::setprecision(0) << std::fixed << msg.hardware_ts() << sep
			  << std::setw(sensor_width) << vec3_to_string(msg.accel()) << sep
			  << std::setw(time_width) << std::setprecision(0) << std::fixed << msg.hardware_ts() << sep
			  << std::setw(sensor_width) << vec3_to_string(msg.gyro()) << sep << "\n";

	std::cout << line << '\n';

	std::cout << std::endl;
}

} // namespace rspub
