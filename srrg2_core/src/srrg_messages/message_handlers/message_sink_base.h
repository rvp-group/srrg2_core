#pragma once
#include <memory>
#include <map>
#include "srrg_boss/serializer.h"
#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"
#include "../messages/base_sensor_message.h"
#include "srrg_data_structures/platform.h"

namespace srrg2_core {

  class ViewerCanvas;
  
  class MessageSinkBase: public Configurable {
  public:
    MessageSinkBase();
    virtual ~MessageSinkBase();
    virtual bool putMessage(BaseSensorMessagePtr msg_) = 0;
    inline bool isOpen() {return _is_open;}
  protected:
    bool _is_open = false;
  };

  using MessageSinkBasePtr=std::shared_ptr<MessageSinkBase>;
}
