#include "message_pack.h"

namespace srrg2_core {

  MessagePack::MessagePack(const std::string& topic_,
                           const std::string& frame_id_,
                           const int& seq_,
                           const double& timestamp_):
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_) {
  }

  void MessagePack::serialize(ObjectData& odata, IdContext& context) {
    /*
      ArrayData* adata = new ArrayData;
      for (size_t i = 0; i < messages.size(); ++i) {
      adata->add(new PointerData(messages[i].get()));
      }
      odata.setField("messages", adata);
    */
  }

  void MessagePack::deserialize(ObjectData& odata, IdContext& context) {
    /*
      ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField("messages"));
      message_ptrs.resize(adata->size());
      for (size_t i = 0; i < adata->size(); ++i) {
      ValueData& vdata = (*adata)[i];
      vdata.getReference().bind(message_ptrs[i]);
      }
    */
  }

  void MessagePack::deserializeComplete() {
    /*
      messages.clear();
      for (size_t i = 0; i < message_ptrs.size(); ++i) {
      BaseSensorMessage* msg = message_ptrs[i];
      messages.push_back(BaseSensorMessagePtr(msg));
      }
      message_ptrs.clear();
    */
  }
}
