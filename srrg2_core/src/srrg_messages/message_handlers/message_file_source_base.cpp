#include "message_file_source_base.h"
#include "srrg_config/configurable.h"
#include "srrg_system_utils/system_utils.h"
#include <signal.h>
#include <termios.h>

namespace srrg2_core {

  void MessageFileSourceBase::open(const std::string& filename_) {
    if (_is_open) {
      this->close();
    }
    if (!isAccessible(filename_)) {
      std::string msg = filename_ + " not found";
      throw std::runtime_error(msg.c_str());
    }
    param_filename.setValue(filename_);
    open();
    _is_open           = true;
    _file_changed_flag = false;

    // signal(SIGINT, MessageFileSourceBase::sigIntHandler);

    //_pause_thread = std::thread(&MessageFileSourceBase::_checkIsPlaying);
  }

  MessageSourceBase* MessageFileSourceBase::getRootSource() {
    return this;
  }

  void MessageFileSourceBase::close() {
    if (!_is_open) {
      return;
    }
    _is_open = false;
    // while (!_pause_thread.joinable()) {
    //   std::cerr << "waiting for pause thread to be joinable" << std::endl;
    //   std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //   continue;
    // }
    // _pause_thread.join();
  }

  MessageFileSourceBase::~MessageFileSourceBase() {
  }

} // namespace srrg2_core
