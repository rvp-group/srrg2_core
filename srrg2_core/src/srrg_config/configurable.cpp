#include "configurable.h"
#include "configurable_shell.h"
#include <sstream>

namespace srrg2_core {

  class ConfigurableCommandHelp : public Configurable::CommandBase {
  public:
    ConfigurableCommandHelp(Configurable* c) :
      Configurable::CommandBase(c, "help", "lists possible commands for this module") {
    }

    virtual ~ConfigurableCommandHelp() {
    }

    bool execute(ConfigurableShell* shell_,
                 std::string& response,
                 const std::vector<std::string>& tokens) override {
      std::ostringstream os;
      os << "module: " << _configurable->className() << " ptr: " << _configurable << std::endl;
      for (auto& it : _configurable->_command_map) {
        os << "\t" << it.second->tag() << ": " << it.second->helpMessage() << std::endl;
      }
      response = os.str();
      return true;
    }
  };

  Configurable::CommandBase::~CommandBase() {
  }

  Configurable::Configurable() {
    addCommand(new ConfigurableCommandHelp(this));
  }

  void Configurable::addCommand(CommandBase* cmd) {
    if (command(cmd->tag())) {
      throw std::runtime_error("command already in map");
    }
    _command_map.insert(std::make_pair(cmd->tag(), CommandBasePtr(cmd)));
  }

  Configurable::CommandBase* Configurable::command(const std::string& tag_) {
    auto it = _command_map.find(tag_);
    if (it != _command_map.end()) {
      return it->second.get();
    }
    return nullptr;
  }

  bool Configurable::handleCommand(ConfigurableShell* shell_,
                                   std::string& response,
                                   const std::vector<std::string>& tokens) {
    if (!tokens.size()) {
      response = "unknown empty command";
      return false;
    }
    CommandBase* cmd = command(tokens[0]);
    if (!cmd) {
      response = std::string("unknown command[") + tokens[0] + "]";
      return false;
    }
    return cmd->execute(shell_, response, tokens);
  }

} // namespace srrg2_core
