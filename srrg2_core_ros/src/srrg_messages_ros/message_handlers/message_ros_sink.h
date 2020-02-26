#pragma once
#include <ros/ros.h>
#include "srrg_messages/instances.h"
#include "srrg_property/property_vector.h"

namespace srrg2_core_ros {
  class MessageROSSink: public srrg2_core::MessageSinkBase {
  public:
    PARAM_VECTOR(srrg2_core::PropertyVector_<std::string>,
                 topics,
                 "list of topics where to publish",
                 0);
    PARAM_VECTOR(srrg2_core::PropertyVector_<std::string>,
                 type,
                 "list of type for each topic (must have the same lenght and positioning of values)",
                 0);
      public:
    
    MessageROSSink();
    ~MessageROSSink();
    void open();
    void close();

    virtual bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;

  private:
    ros::NodeHandle _nh;
    std::map<std::string, ros::Publisher> _publishers;
  };

}
