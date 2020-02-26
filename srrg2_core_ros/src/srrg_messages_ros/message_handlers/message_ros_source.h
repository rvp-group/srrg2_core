#pragma once
//srrg ros stuff
#include <ros/ros.h>
#include "srrg_converters/converter.h"

//srrg srrg2 stuff
#include "srrg_messages/instances.h"

namespace srrg2_core_ros {
  void rosInit();
  
  class MessageROSSource: public srrg2_core::MessageSourceBase {
  public:
    PARAM_VECTOR(srrg2_core::PropertyVector_<std::string>, topics, "list of the topics you want to convert",  0);
    PARAM(srrg2_core::PropertyBool, verbose, "verbose", false, 0);
    
    MessageROSSource();
    ~MessageROSSource();

//ds interface
  public:

    MessageSourceBase* getRootSource() override;
    srrg2_core::BaseSensorMessagePtr getMessage() override;
    void open() override;
    void close();

    bool ok() {
      return ros::ok();
    }

//ds helpers
  protected:

    void _cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg_);
    void _imageCallback(const sensor_msgs::ImageConstPtr& msg_);
    void _imuCallback(const sensor_msgs::ImuConstPtr& msg_);
    void _laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg_);
    void _odometryCallback(const nav_msgs::OdometryConstPtr& msg_);
    void _pointStampedCallback(const geometry_msgs::PointStampedConstPtr& msg_);
    void _rangeCallback(const sensor_msgs::RangeConstPtr& msg_);
    void _tfMessageCallback(const tf2_msgs::TFMessageConstPtr& msg_);
    void _twistStampedCallback(const geometry_msgs::TwistStampedConstPtr& msg_);
    void _pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg_);

  protected:

    //srrg the map holds the messages by timestamp
    std::multimap<double, srrg2_core::PropertyContainerSerializablePtr> _message_queue;
    ros::NodeHandle _node_handle;
    std::vector<ros::Subscriber> _subscribers;
    ros::master::V_TopicInfo topics_to_process;
    uint64_t _sequence_number = 0;
  };
}
