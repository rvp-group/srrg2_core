#pragma once
#include "message_source_base.h"
#include "srrg_config/property_configurable.h"
namespace srrg2_core {

  class MessageFilterBase: public MessageSourceBase {
  public:

    PARAM(PropertyConfigurable_<MessageSourceBase>, source, "the source from where this filter reads", 0, 0);

  public:
  virtual ~MessageFilterBase();
    MessageSourceBase* getRootSource() override ;
    BaseSensorMessagePtr getMessage() = 0;
  protected:
  };

  using MessageFilterBasePtr=std::shared_ptr<MessageFilterBase>;

}
