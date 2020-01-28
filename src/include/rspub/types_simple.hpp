
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>

#include "PoseMessage.pb.h"
#include "ImageMessage.pb.h"
#include "IMUMessage.pb.h"
#include "PointCloudMessage.pb.h"

namespace rspub
{

using PubImage = eCAL::protobuf::CPublisher<rspub_pb::ImageMessage>;
using PubPose = eCAL::protobuf::CPublisher<rspub_pb::PoseMessage>;
using PubPointCloud = eCAL::protobuf::CPublisher<rspub_pb::PointCloudMessage>;

using PubImage = eCAL::protobuf::CPublisher<rspub_pb::ImageMessage>;
using PubPose = eCAL::protobuf::CPublisher<rspub_pb::PoseMessage>;
using PubPointCloud = eCAL::protobuf::CPublisher<rspub_pb::PointCloudMessage>;

using SubImage = eCAL::protobuf::CSubscriber<rspub_pb::ImageMessage>;
using SubPose = eCAL::protobuf::CSubscriber<rspub_pb::PoseMessage>;
using SubPointCloud = eCAL::protobuf::CSubscriber<rspub_pb::PointCloudMessage>;

}