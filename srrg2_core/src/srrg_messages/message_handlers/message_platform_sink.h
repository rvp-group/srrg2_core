#pragma once
#include "srrg_data_structures/platform.h"
#include "message_sink_base.h"

namespace srrg2_core {

  class ViewerCanvas;
  
  class MessagePlatformSink: public MessageSinkBase, public PlatformUser {
  public:
    PARAM(PropertyString, tf_topic, "name of the transform tree to subscribe to", "/tf", 0);
    
    virtual bool putMessage(BaseSensorMessagePtr msg_) = 0;
  };

  using MessagePlatformSinkPtr=std::shared_ptr<MessagePlatformSink>;
}
