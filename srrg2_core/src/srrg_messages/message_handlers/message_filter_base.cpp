#include "message_filter_base.h"
namespace srrg2_core {


  MessageFilterBase::~MessageFilterBase(){}

  MessageSourceBase* MessageFilterBase::getRootSource() {
    if (param_source.value())
      return param_source->getRootSource();
    return 0;
  }
}
