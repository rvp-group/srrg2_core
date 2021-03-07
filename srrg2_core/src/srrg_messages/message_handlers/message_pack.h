#pragma once
#include "../messages/base_sensor_message.h"

namespace srrg2_core {

  // a message pack "owns" its messages
  class MessagePack : public BaseSensorMessage {
  public:
    MessagePack(const std::string& topic_    = "",
                const std::string& frame_id_ = "",
                const int& seq_              = -1,
                const double& timestamp_     = -1.0);

    std::vector<BaseSensorMessagePtr> messages;
    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;
    void deserializeComplete() override;

  protected:
    // used during deserialization
    std::vector<BaseSensorMessage*> message_ptrs;
  };

  typedef std::shared_ptr<MessagePack> MessagePackPtr;
} // namespace srrg2_core
