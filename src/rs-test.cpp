// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
// Realsense Tracking Module

// This technique works with RSUSB driver (LIBUVC) but should work with kernel drivers as well
// This example is meant for D435i, to get high frequency motion data at the same time of depth,rgb images
// Two examples are shown, one that read a bags file and another that does live streaming from the sensor. Activate bag reading with cmd line flag --bag=<file>
// The main technique that makes this possible is to NOT USE rs::pipeline, but to directly acess each sensor and 
// configure the stream manually (as well as associated callbacks).

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
#include <thread>

#include <librealsense2/rs.hpp>
#include <gflags/gflags.h>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>

#include "utility.hpp"
#include "ImageData.pb.h"
#include "IMUMessage.pb.h"

using namespace std::chrono_literals;


void publish()
{
	eCAL::CPublisher pub("Counter", "long long");
	// idle main thread
	while(eCAL::Ok())
	{
		// sleep 100 ms
		std::cout << "Sending data on seperate thread" << std::endl;
		long long start_time_loop = eCAL::Time::GetMicroSeconds();
		pub.Send(&start_time_loop, sizeof(long long));
		eCAL::Process::SleepMS(1000);
	}

}
void OnReceive(const char* /* topic_name_ */, const struct eCAL::SReceiveCallbackData* data_)
{
	std::string rec_buf;
	std::cout << "Received data on main thread: " << std::endl;
}

int main(int argc, char* argv[])
{
	  // initialize eCAL API
	eCAL::Initialize(0, nullptr, "RSTrackerSub");
    eCAL::CSubscriber sub("Counter", "long long");

  	// setup receive callback function
  	sub.AddReceiveCallback(OnReceive);

	auto t1 = std::thread(publish);
  	// idle main thread
	while(eCAL::Ok())
	{
		// sleep 100 ms
		std::cout << "Main thread loop waiting for messages" << std::endl;
		eCAL::Process::SleepMS(1000);
	}

}


// namespace rstracker
// {


// void OnIMUMessage(const char* topic_name_, const rstracker_pb::IMUMessage& imu_msg, const long long time_, const long long clock_)
// {
// 	std::cout<< std::setprecision(0) << std::fixed << "Received IMUMessage; ts: " << imu_msg.ts() << std::endl;

// }


// }

// int main(int argc, char* argv[]) try
// {
// 	  // initialize eCAL API
// 	eCAL::Initialize(0, nullptr, "RSTrackerSub");

// 	// create subscriber
// 	std::cout << "To here" << std::endl;

// 	eCAL::Process::SleepMS(2000);
// 	eCAL::protobuf::CSubscriber<rstracker_pb::IMUMessage> sub_imu("IMUMessage");
//   	auto imu_rec_callback = std::bind(rstracker::OnIMUMessage, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
// 	sub_imu.AddReceiveCallback(imu_rec_callback);
// 	std::cout << "Before main loop" << std::endl;
// 	while(eCAL::Ok())
// 	{
// 		// sleep 100 ms
// 		eCAL::Process::SleepMS(10);
// 		// std::cout<< "Test" <<std::endl;
// 	}
	
	
// }
// catch (const rs2::error & e)

// {

// 	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;

// 	return EXIT_FAILURE;
// }

// catch (const std::exception & e)

// {

// 	std::cerr << e.what() << std::endl;

// 	return EXIT_FAILURE;
// }


