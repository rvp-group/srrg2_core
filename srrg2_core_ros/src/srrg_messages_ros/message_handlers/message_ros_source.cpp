#include "message_ros_source.h"
#include <srrg_system_utils/system_utils.h>
#include <srrg_config/configurable_command.h>
#include <sys/types.h>
#include <unistd.h>

namespace srrg2_core_ros {

  static bool ros_initialized = false;
  void rosInit() {
    if (!srrg2_core::srrg_argc) {
      throw std::runtime_error("rosInit| missing call to srrgInit. Aborting.");
    }
    if (!ros_initialized) {
      ros::init(srrg2_core::srrg_argc,
                srrg2_core::srrg_argv,
                srrg2_core::srrg_opts.c_str(),
                ros::InitOption::NoSigintHandler|ros::InitOption::AnonymousName);
    }
    ros_initialized = true;
  }

  MessageROSSource::MessageROSSource() {
    addCommand (new srrg2_core::ConfigurableCommand_
                < MessageROSSource,
                typeof(&MessageROSSource::cmdOpen),
                std::string>
                (this,
                 "open",
                 "starts receiving messages",
                 &MessageROSSource::cmdOpen));

    addCommand (new srrg2_core::ConfigurableCommand_
                < MessageROSSource,
                typeof(&MessageROSSource::cmdClose),
                std::string>
                (this,
                 "close",
                 "starts receiving messages",
                 &MessageROSSource::cmdClose));
    rosInit();
    _message_queue.clear();
    _subscribers.clear();
    topics_to_process.clear();
  }

  bool MessageROSSource::cmdOpen(std::string& response) {
    open();
    response = "source opened";
    return true;
  }

  bool MessageROSSource::cmdClose(std::string& response) {
    close();
    response = "source closed";
    return true;
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
        _subscribers.push_back(_node_handle.subscribe<sensor_msgs::CameraInfo>
                               (ttp.name, 1, boost::bind(&MessageROSSource::_cameraInfoCallback, this, _1, ttp.name) ) );
      } else if (ttp.datatype == "sensor_msgs/Image") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<sensor_msgs::Image>
                               (ttp.name, 1, boost::bind(&MessageROSSource::_imageCallback, this, _1, ttp.name) ) );
      } else if (ttp.datatype == "sensor_msgs/Imu") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<sensor_msgs::Imu>
                               (ttp.name, 1, boost::bind(&MessageROSSource::_imuCallback, this, _1, ttp.name ) ) );
      } else if (ttp.datatype == "nav_msgs/Odometry") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<nav_msgs::Odometry>
                               (ttp.name, 1, boost::bind(&MessageROSSource::_odometryCallback, this, _1, ttp.name ) ) );
      } else if (ttp.datatype == "geometry_msgs/PointStamped") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<geometry_msgs::PointStamped>
                               (ttp.name, 1, boost::bind(&MessageROSSource::_pointStampedCallback, this, _1, ttp.name ) ) );
      } else if (ttp.datatype == "sensor_msgs/LaserScan") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<sensor_msgs::LaserScan>
                               (ttp.name, 1, boost::bind(&MessageROSSource::_laserScanCallback, this, _1, ttp.name ) ) );
      } else if (ttp.datatype == "sensor_msgs/Range") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<sensor_msgs::Range>
                               (ttp.name, 1, boost::bind(&MessageROSSource::_rangeCallback, this, _1, ttp.name ) ) );
      } else if (ttp.datatype == "tf2_msgs/TFMessage") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<tf2_msgs::TFMessage>
                               (ttp.name, 20, boost::bind(&MessageROSSource::_tfMessageCallback, this, _1, ttp.name ) ) );
      } else if (ttp.datatype == "geometry_msgs/TwistStamped") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<geometry_msgs::TwistStamped>
                               (ttp.name, 1, boost::bind(&MessageROSSource::_twistStampedCallback, this, _1, ttp.name ) ) );
      } else if (ttp.datatype == "sensor_msgs/PointCloud2") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<sensor_msgs::PointCloud2>
                               (ttp.name, 1, boost::bind(&MessageROSSource::_pointCloud2Callback, this, _1, ttp.name ) ) );
      } else if (ttp.datatype == "geometry_msgs/PoseWithCovarianceStamped") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<geometry_msgs::PoseWithCovarianceStamped>
                               (ttp.name, 100, boost::bind(&MessageROSSource::_poseWithCovarianceStampedCallback, this, _1, ttp.name ) ) );
      } else if (ttp.datatype == "geometry_msgs/PoseStamped") {
        ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages"
           << std::endl;
        _subscribers.push_back(_node_handle.subscribe<geometry_msgs::PoseStamped>
                               (ttp.name, 100, boost::bind(&MessageROSSource::_poseStampedCallback, this, _1, ttp.name ) ) );
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
    while (_message_queue.empty() && _is_open) {
      ros::spinOnce();
      usleep(10000);
    }
    if (! _is_open)
      return nullptr;
    
    auto it=_message_queue.begin();
    srrg2_core::BaseSensorMessagePtr returned=it->second;
    _message_queue.erase(it);
    if (returned)
      return returned;
    return nullptr;
  }

  
  void MessageROSSource::_cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_imageCallback(const sensor_msgs::ImageConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_imuCallback(const sensor_msgs::ImuConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_odometryCallback(const nav_msgs::OdometryConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_pointStampedCallback(const geometry_msgs::PointStampedConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_rangeCallback(const sensor_msgs::RangeConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_tfMessageCallback(const tf2_msgs::TFMessageConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg_->transforms[0].header.stamp.toSec(), msg));
  }

  void MessageROSSource::_twistStampedCallback(const geometry_msgs::TwistStampedConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    std::cerr << "received PoseStamped , topic: [" << topic_name << "]" << std::endl;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_poseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    std::cerr << "received PoseWithCovarianceStamped , topic: [" << topic_name << "]" << std::endl;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg_, const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg=Converter::convert(msg_);
    if (! msg)
      return;
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  srrg2_core::MessageSourceBase* MessageROSSource::getRootSource() {
    return this;
  }

  MessageROSSource::~MessageROSSource() {
    if (_is_open) {
      this->close();
    }
    if (_node_handle.ok()) {
      _node_handle.shutdown();
    }
  }

} // namespace srrg2_core_ros
