#include "message_ros_sink.h"
#include "srrg_converters/converter.h"
#include "message_ros_source.h"

#define CONVERT(SRRG_MESSAGE)                                                               \
  {                                                                                         \
    if (auto ros_msg = srrg2_core_ros::Converter::convert(std::dynamic_pointer_cast<SRRG_MESSAGE>(msg_))) { \
      pubs_it->second.publish(ros_msg);                                                     \
      return true;                                                                          \
    }                                                                                       \
  }

#define ADVERTISE_TOPIC(SRRG_CLASS_NAME, ROS_MESSAGE)                                \
{                                                                                    \
  const std::string& topic = param_topics.value(i);                              \
  if (param_type.value(i) == #SRRG_CLASS_NAME) {                                 \
    _publishers.insert(std::make_pair(topic, _nh.advertise<ROS_MESSAGE>(topic, 1))); \
    continue;                                                                        \
  }                                                                                  \
}
using namespace srrg2_core;

namespace srrg2_core_ros {

  MessageROSSink::MessageROSSink()
  {
    rosInit();
  }

  void MessageROSSink::open() {
    rosInit();
    this->close();
    const std::size_t size = param_type.size();

    if (size != param_topics.size()) {
      throw std::runtime_error("topics and types have different length");
    }

    for (std::size_t i = 0; i < size; ++i) {

      ADVERTISE_TOPIC(CameraInfoMessage, sensor_msgs::CameraInfo);
      ADVERTISE_TOPIC(ImageMessage, sensor_msgs::Image);
      ADVERTISE_TOPIC(IMUMessage, sensor_msgs::Imu);
      ADVERTISE_TOPIC(LaserMessage, sensor_msgs::LaserScan);
      ADVERTISE_TOPIC(RangeMessage, sensor_msgs::Range);
      ADVERTISE_TOPIC(TransformEventsMessage, tf2_msgs::TFMessage);
      ADVERTISE_TOPIC(OdometryMessage, nav_msgs::Odometry);
      ADVERTISE_TOPIC(PointStampedMessage, geometry_msgs::PointStamped);
      ADVERTISE_TOPIC(TwistStampedMessage, geometry_msgs::TwistStamped);
      ADVERTISE_TOPIC(PointCloud2Message, sensor_msgs::PointCloud2);


    }

    _is_open = true;
  }

  void MessageROSSink::close() {
    if (!_is_open) {
      return;
    }
    for (auto& p: _publishers) {
      p.second.shutdown();
    }
    _is_open = false;
  }

  bool MessageROSSink::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {

    if (!msg_) {
      return false;
    }

    auto pubs_it = _publishers.find(msg_->topic.value());
    if (pubs_it == _publishers.end()) {
      return false;
    }

    CONVERT(srrg2_core::CameraInfoMessage);
    CONVERT(srrg2_core::ImageMessage);
    CONVERT(srrg2_core::IMUMessage);
    CONVERT(srrg2_core::LaserMessage);
    CONVERT(srrg2_core::RangeMessage);
    CONVERT(srrg2_core::TransformEventsMessage);
    CONVERT(srrg2_core::OdometryMessage);
    CONVERT(srrg2_core::PointStampedMessage);
    CONVERT(srrg2_core::TwistStampedMessage);
    CONVERT(srrg2_core::PointCloud2Message);

    return false;
  }

  MessageROSSink::~MessageROSSink() {
    if (_is_open) {
      this->close();
    }
  }

}
