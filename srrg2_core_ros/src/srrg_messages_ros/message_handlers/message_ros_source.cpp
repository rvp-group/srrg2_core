#include "message_ros_source.h"
#include <srrg_system_utils/system_utils.h>

namespace srrg2_core_ros {

  static bool ros_initialized = false;
  void rosInit() {
    if (!srrg2_core::srrg_argc) {
      throw std::runtime_error("rosInit| missing call to srrgInit. Aborting.");
    }
    if (!ros_initialized) {
      ros::init(srrg2_core::srrg_argc, srrg2_core::srrg_argv, srrg2_core::srrg_opts.c_str());
    }
    ros_initialized = true;
  }

  MessageROSSource::MessageROSSource() {
    rosInit();
    _message_queue.clear();
    _subscribers.clear();
    topics_to_process.clear();
  }

  void MessageROSSource::open() {
    rosInit();
    ros::master::V_TopicInfo all_topics;
    std::cerr << "waiting for new topics";
    ros::Rate r(10);
    int count = 0;
    while (all_topics.size() < 3 && count < 100) {
      ros::master::getTopics(all_topics);
      r.sleep();
      if (!(count % 10)) {
        std::cerr << ".";
      }
      count++;
    }
    if (count >= 100) {
      std::cerr << " no interesting topics" << std::endl;
      _is_open = false;
      return;
    }
    r.sleep();
    ros::master::getTopics(all_topics);

    if (param_verbose.value()) {
      std::cerr << "MessageROSSource::open|number of available ROS topics: " << all_topics.size()
                << std::endl;
    }

    if (param_topics.value().size() == 0) {
      topics_to_process = all_topics;
    } else {
      // srrg TODO this sucks -> find a better way
      for (const std::string& topic_name : param_topics.value()) {
        // srrg TopicInfo = struct {name, datatype}
        for (const ros::master::TopicInfo& other_topic : all_topics) {
          if (topic_name == other_topic.name) {
            topics_to_process.push_back(other_topic);
          }
        }
      }
    }

    // srrg qui avviene la merda
    std::stringstream ss;
    for (const ros::master::TopicInfo& ttp : topics_to_process) {
      ss << "MessageROSSource::open|";
      if (ttp.datatype == "sensor_msgs/CameraInfo") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(
          _node_handle.subscribe(ttp.name, 1, &MessageROSSource::_cameraInfoCallback, this));
      } else if (ttp.datatype == "sensor_msgs/Image") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(
          _node_handle.subscribe(ttp.name, 1, &MessageROSSource::_imageCallback, this));
      } else if (ttp.datatype == "sensor_msgs/Imu") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(
          _node_handle.subscribe(ttp.name, 1, &MessageROSSource::_imuCallback, this));
      } else if (ttp.datatype == "nav_msgs/Odometry") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(
          _node_handle.subscribe(ttp.name, 1, &MessageROSSource::_odometryCallback, this));
      } else if (ttp.datatype == "geometry_msgs/PointStamped") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(
          _node_handle.subscribe(ttp.name, 1, &MessageROSSource::_pointStampedCallback, this));
      } else if (ttp.datatype == "sensor_msgs/LaserScan") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(
          _node_handle.subscribe(ttp.name, 1, &MessageROSSource::_laserScanCallback, this));
      } else if (ttp.datatype == "sensor_msgs/Range") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(
          _node_handle.subscribe(ttp.name, 1, &MessageROSSource::_rangeCallback, this));
      } else if (ttp.datatype == "tf2_msgs/TFMessage") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(
          _node_handle.subscribe(ttp.name, 1, &MessageROSSource::_tfMessageCallback, this));
      } else if (ttp.datatype == "geometry_msgs/TwistStamped") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(
          _node_handle.subscribe(ttp.name, 1, &MessageROSSource::_twistStampedCallback, this));
      } else if (ttp.datatype == "sensor_msgs/PointCloud2") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(
          _node_handle.subscribe(ttp.name, 1, &MessageROSSource::_pointCloud2Callback, this));
      } else {
        ss << "unable to convert " << ttp.datatype << " messages (no topic registered)"
           << std::endl;
      }
      if (param_verbose.value()) {
        std::cerr << ss.str() << std::endl;
        ss.clear();
      }
    }
    if (param_verbose.value()) {
      std::cerr << "subscribed to " << _subscribers.size() << " topics" << std::endl;
    }

    _is_open = true;
  }

  void MessageROSSource::close() {
    if (!_is_open) {
      return;
    }
    for (ros::Subscriber& s : _subscribers) {
      s.shutdown();
    }
    _message_queue.clear();
    _subscribers.clear();
    topics_to_process.clear();
    _is_open = false;
  }

  srrg2_core::BaseSensorMessagePtr MessageROSSource::getMessage() {
    _message_queue.clear();
    ros::spinOnce();
    if (_message_queue.empty()) {
      return 0;
    }

    // srrg we use message pack since it is possible that more messages arrives in a single
    //     spin, thus we encapsule everything in a pack
    srrg2_core::MessagePackPtr pack =
      srrg2_core::MessagePackPtr(new srrg2_core::MessagePack("", "", _sequence_number, 0));
    for (auto msg : _message_queue) {
      pack->messages.push_back(
        std::dynamic_pointer_cast<srrg2_core::BaseSensorMessage>(msg.second));
    }
    ++_sequence_number;
    _message_queue.clear();
    return pack;
  }

  void MessageROSSource::_cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg_) {
    _message_queue.insert(std::make_pair(msg_->header.stamp.toSec(), Converter::convert(msg_)));
  }

  void MessageROSSource::_imageCallback(const sensor_msgs::ImageConstPtr& msg_) {
    _message_queue.insert(std::make_pair(msg_->header.stamp.toSec(), Converter::convert(msg_)));
  }

  void MessageROSSource::_imuCallback(const sensor_msgs::ImuConstPtr& msg_) {
    _message_queue.insert(std::make_pair(msg_->header.stamp.toSec(), Converter::convert(msg_)));
  }

  void MessageROSSource::_odometryCallback(const nav_msgs::OdometryConstPtr& msg_) {
    _message_queue.insert(std::make_pair(msg_->header.stamp.toSec(), Converter::convert(msg_)));
  }

  void MessageROSSource::_pointStampedCallback(const geometry_msgs::PointStampedConstPtr& msg_) {
    _message_queue.insert(std::make_pair(msg_->header.stamp.toSec(), Converter::convert(msg_)));
  }

  void MessageROSSource::_laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg_) {
    _message_queue.insert(std::make_pair(msg_->header.stamp.toSec(), Converter::convert(msg_)));
  }

  void MessageROSSource::_rangeCallback(const sensor_msgs::RangeConstPtr& msg_) {
    _message_queue.insert(std::make_pair(msg_->header.stamp.toSec(), Converter::convert(msg_)));
  }

  void MessageROSSource::_tfMessageCallback(const tf2_msgs::TFMessageConstPtr& msg_) {
    _message_queue.insert(
      std::make_pair(msg_->transforms[0].header.stamp.toSec(), Converter::convert(msg_)));
  }

  void MessageROSSource::_twistStampedCallback(const geometry_msgs::TwistStampedConstPtr& msg_) {
    _message_queue.insert(std::make_pair(msg_->header.stamp.toSec(), Converter::convert(msg_)));
  }

  void MessageROSSource::_pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg_) {
    _message_queue.insert(std::make_pair(msg_->header.stamp.toSec(), Converter::convert(msg_)));
  }

  srrg2_core::MessageSourceBase* MessageROSSource::getRootSource() {
    return this;
  }

  MessageROSSource::~MessageROSSource() {
    if (_is_open) {
      this->close();
    }
  }

} // namespace srrg2_core_ros
