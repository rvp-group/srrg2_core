#pragma once
#include <thread>
#include "srrg_config/configurable.h"
#include "message_source_base.h"
#include "srrg_property/property.h"
namespace srrg2_core {

  class MessageFileSourceBase: public MessageSourceBase {

  // configuration
  public:

    PARAM(PropertyString, filename, "file to read", "", &_file_changed_flag);
  public:


    MessageFileSourceBase()  { _is_open = false; }
  virtual ~MessageFileSourceBase();

    MessageSourceBase* getRootSource() override;
    
    virtual void open(const std::string& filename);
    virtual void open() override = 0;
    virtual void close();

  protected:
    bool _file_changed_flag = true;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using MessageFileSourceBasePtr = std::shared_ptr<MessageFileSourceBase>;

}
