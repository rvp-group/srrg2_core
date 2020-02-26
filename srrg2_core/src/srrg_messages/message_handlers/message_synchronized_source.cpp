#include "message_synchronized_source.h"
#include "message_pack.h"

namespace srrg2_core {

  void MessageSynchronizedSource::clearBuffer() {
    // we delete all messages in the buffers
    for (auto it = _message_map.begin(); it != _message_map.end(); ++it) {
      if (it->second) {
        it->second.reset();
      }
    }
  }

  void MessageSynchronizedSource::handleTopicsChanged() {
    resetCounters();
    clearBuffer();
    _message_map.clear();
    for (size_t i = 0; i < param_topics.size(); ++i) {
      const std::string& t = param_topics.value(i);
      _message_map.insert(std::make_pair(t, nullptr));
    }
    _topics_changed = false;
    _seq            = 0;
  }

  void MessageSynchronizedSource::handleIntervalChanged() {
    resetCounters();
    _interval_changed = false;
  }

  BaseSensorMessagePtr MessageSynchronizedSource::getMessage() {
    assert(param_source.value() && "you need to set a valid source");
    MessageSourceBasePtr src = param_source.value();

    if (_topics_changed) {
      // cerr << "topics changed" << endl;
      handleTopicsChanged();
    }
    if (_interval_changed) {
      // cerr << "interval changed" << endl;
      handleIntervalChanged();
    }
    while (!isPacketReady()) {
      BaseSensorMessagePtr msg = src->getMessage();
      if (!msg) {
        // cerr << __PRETTY_FUNCTION__ << ": source ova" << endl;
        return 0;
      }
      // cerr << "got msg " << msg->topic.value() << endl;

      // find a bucket;
      auto it = _message_map.find(msg->topic.value());
      if (it == _message_map.end()) {
        // cerr << "msg not map, deleting " << endl;
        // delete msg;
        msg.reset();
        continue;
      }

      // we assume the message is more recent than the previous
      // one and we replace it
      if (it->second) {
        // cerr << "replacing old msg "
        //     << it->second->timestamp.value() << endl;
        ++_num_dropped_messages;
        // delete it->second;
        it->second.reset();
      }
      it->second = msg;
      // cerr << "new ts: "
      //   << it->second->timestamp.value() << endl;
    }
    // we come here if all messages are synchronized
    // we need to assemble them in a packet
    // in the order of the topics
    MessagePack* pack =
      new MessagePack(param_output_topic.value(), param_output_frame_id.value(), _seq, _t_min);
    for (size_t i = 0; i < param_topics.size(); ++i) {
      const std::string& t = param_topics.value(i);
      auto it              = _message_map.find(t);
      assert(it != _message_map.end() && "no messge in map");
      assert(it->second && "invalid pointer");
      pack->messages.push_back(BaseSensorMessagePtr(it->second));
      it->second = 0;
    }
    ++_seq;
    return BaseSensorMessagePtr(pack);
  }

  bool MessageSynchronizedSource::isPacketReady() {
    _t_min = std::numeric_limits<double>::max();
    _t_max = 0;
    // a pack is ready if the lag between t_max and t_min is below time_interval
    for (auto it = _message_map.begin(); it != _message_map.end(); ++it) {
      if (!it->second) {
        return false;
      }
      _t_min = std::min(_t_min, it->second->timestamp.value());
      _t_max = std::max(_t_max, it->second->timestamp.value());
    }
    return (_t_max - _t_min) < param_time_interval.value();
  }

  void MessageSynchronizedSource::resetCounters() {
    _num_dropped_messages = 0;
  }
} // namespace srrg2_core
