#include "message_source_platform.h"
#include <typeinfo>

namespace srrg2_core {

  MessageSourcePlatform::~MessageSourcePlatform() {
  }

  void MessageSourcePlatform::bindTfTopics() {
    _tf_topics_binded = false;
    for (const std::string& t : param_tf_topics.value()) {
      StringPlatformPtrMap::iterator p_it = _platforms_map.find(t);
      if (p_it == _platforms_map.end()) {
        std::cerr << "MessageSourcePlatform::bindTfTopics|binding TF topic " << FG_YELLOW(t)
                  << " to a transform tree\n";
        _platforms_map.insert(std::make_pair(t, PlatformPtr(new Platform)));
      }
    }
    _tf_topics_binded = true;
    return;
  }

  BaseSensorMessagePtr MessageSourcePlatform::getMessage() {
    assert(param_source.value() &&
           "MessageSourcePlatform::getMessage|you need to set a valid source");

    if (!_tf_topics_binded) {
      // throw std::runtime_error("MessageSourcePlatform::getMessage|did you forget to call the
      // bindTfTopics function?");
      bindTfTopics();
    }

    BaseSensorMessagePtr msg = param_source->getMessage();
    // ia source ended
    if (!msg) {
      std::cerr << "MessageSourcePlatform::getMessage|source ended" << std::endl;
      return nullptr;
    }

    //    std::cerr << "MessageSourcePlatform::getMessage|typeid hash_code= " <<
    //    FG_YELLOW(typeid(*(msg.get())).hash_code()) << std::endl; std::cerr <<
    //    "MessageSourcePlatform::getMessage|typeid name     = " <<
    //    FG_YELLOW(typeid(*(msg.get())).name()) << std::endl; std::cerr <<
    //    "MessageSourcePlatform::getMessage|boss class_name = " << FG_RED(msg->className()) <<
    //    std::endl;

    StringPlatformPtrMap::const_iterator p_it = _platforms_map.find(msg->topic.value());
    if (p_it != _platforms_map.end() && (!p_it->second->add(msg))) {
      std::cerr << FG_RED("problems with topic: ");
      std::cerr << FG_BLUE(msg->topic.value()) << std::endl;
      throw std::runtime_error("MessageSourcePlatform::getMessage|invalid TF topic");
      //      std::cerr << "MessageSourcePlatform::getMessage|platform " << p_it->first << " size =
      //      " << p_it->second->size() << std::endl;
    }
    return msg;
  }

} /* namespace srrg2_core */
