#include <csignal>
#include <dirent.h>
#include <dlfcn.h>
#include <gnu/lib-names.h>
#include <iomanip>
#include <sstream>
#include <sys/types.h>
#include <ucontext.h>
#include <unistd.h>

#include "srrg_boss/json_object_writer.h"
#include "srrg_config/property_configurable_vector.h"
#include "srrg_system_utils/system_utils.h"

#include "srrg_messages/message_handlers/message_file_source.h"
#include "srrg_messages/message_handlers/message_filter_base.h"
#include "srrg_messages/message_handlers/message_platform_sink.h"
#include "srrg_messages/message_handlers/message_sink_base.h"
#include "srrg_messages/message_handlers/message_source_platform.h"

#include "configurable_shell.h"
#include "linenoise.h"

namespace srrg2_core {

  std::atomic<bool>* ConfigurableShell::_ready = nullptr;

  struct CommandHelp : public ConfigurableShell::CommandBase {
    CommandHelp(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "help", "lists available commands") {
    }
    virtual ~CommandHelp() {
    }
    bool execute(const std::vector<std::string>& args) override {
      std::cerr << FG_BCYAN("COMMANDS") << std::endl;

      for (auto& it : shell->_command_map) {
        std::cerr << std::left << std::setw(6) << " " << CYAN << std::setw(18) << it.first << RESET
                  << it.second->help_message << std::endl;
      }
      return true;
    }
  };

  struct CommandLs : public ConfigurableShell::CommandBase {
    CommandLs(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "ls", "lists the files in the current directory") {
    }
    virtual ~CommandLs() {
    }
    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confFileCompletion(completions, args[0], true, false);
    }

    bool execute(const std::vector<std::string>& args) override {
      StringVector paths;
      std::string prefix = "";
      if (args.size() > 0)
        prefix = args[0];
      shell->confFileCompletion(paths, prefix);
      for (const auto& it : paths) {
        std::cerr << it << std::endl;
      }
      return true;
    }
  };

  struct CommandPwd : public ConfigurableShell::CommandBase {
    CommandPwd(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "pwd", "prints working directory") {
    }
    virtual ~CommandPwd() {
    }

    bool execute(const std::vector<std::string>& args) override {
      std::cerr << "current_dir:[" << getcwd(0, 0) << "]" << std::endl;
      return true;
    }
  };

  struct CommandCd : public ConfigurableShell::CommandBase {
    CommandCd(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "cd", "changes the working directory") {
    }
    virtual ~CommandCd() {
    }
    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confFileCompletion(completions, args[0], true, false);
    }

    bool execute(const std::vector<std::string>& args) override {
      StringVector paths;
      std::string prefix = "";
      if (args.size() > 0)
        prefix = args[0];
      if (chdir(prefix.c_str())) {
        std::cerr << "error in changing directory" << std::endl;
        return false;
      }
      std::cerr << "current dir is [" << getcwd(0, 0) << "]" << std::endl;
      return true;
    }
  };

  struct CommandDLConfig : public ConfigurableShell::CommandBase {
    CommandDLConfig(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(
        shell_,
        "dl_config",
        "dl_config <filename>, dynamically loads a library from a stub file") {
    }

    virtual ~CommandDLConfig() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confFileCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      std::string filepath(args[0]);
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '\''), filepath.end());
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '"'), filepath.end());
      std::cerr << "opening library [" << FG_YELLOW(filepath) << "]" << std::endl;
      ConfigurableManager::initFactory(filepath);
      return true;
    }
  };

  struct CommandDLOpen : public ConfigurableShell::CommandBase {
    CommandDLOpen(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_,
                                     "dlopen",
                                     "dlopen <library filename>, dynamically loads a library") {
    }
    virtual ~CommandDLOpen() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confFileCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      std::string filepath(args[0]);
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '\''), filepath.end());
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '"'), filepath.end());
      std::cerr << "opening library [" << FG_YELLOW(filepath) << "]" << std::endl;
      void* handle = dlopen(filepath.c_str(), RTLD_LAZY);
      if (!handle) {
        std::cerr << FG_RED(*dlerror()) << std::endl;
        return false;
      }
      ConfigurableManager::initFactory();
      return true;
    }
  };

  struct CommandOpen : public ConfigurableShell::CommandBase {
    CommandOpen(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "open", "open <filename>, opens a file") {
    }
    virtual ~CommandOpen() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confFileCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      std::string filepath(args[0]);
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '\''), filepath.end());
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '"'), filepath.end());
      std::cerr << "opening file [" << FG_YELLOW(filepath) << "]" << std::endl;
      shell->_manager.read(filepath);
      return true;
    }
  };

  struct CommandNames : public ConfigurableShell::CommandBase {
    CommandNames(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "names", "lists the named objects in the manager") {
    }
    virtual ~CommandNames() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confTypesCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 0) {
        return false;
      }
      std::cerr << "named objects: " << std::endl;
      for (auto& map_it : shell->_manager.namedInstances()) {
        ConfigurablePtr c = std::dynamic_pointer_cast<Configurable>(map_it.second);
        if (!c) {
          continue;
        }
        if (args.size() == 1 && args[0] != c->className()) {
          continue;
        }

        std::stringstream ss;
        ss << c.get();

        std::cerr << "ptr: " << ULCYAN << ss.str() << RESET << " class: " << std::left
                  << std::setw(70) << c->className() << " name: " << ULCYAN << c->name() << RESET
                  << std::endl;
      }
      return true;
    }
  };

  struct CommandWrite : public ConfigurableShell::CommandBase {
    CommandWrite(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "write", "<filename>, writes a conf file") {
    }
    virtual ~CommandWrite() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      std::cerr << "writing file [" << FG_YELLOW(args[0]) << "]" << std::endl;
      shell->_manager.write(args[0]);
      return true;
    }
  };

  struct CommandClear : public ConfigurableShell::CommandBase {
    CommandClear(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "clear", "deletes everything") {
    }
    virtual ~CommandClear() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 0) {
        return false;
      }
      shell->clear();
      return true;
    }
  };

  struct CommandQuit : public ConfigurableShell::CommandBase {
    CommandQuit(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "quit", "terminares shell") {
    }
    virtual ~CommandQuit() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size()) {
        return false;
      }
      std::cerr << FG_RED("quitting...") << std::endl;
      shell->_run = false;
      return true;
    }
  };

  struct CommandInstances : public ConfigurableShell::CommandBase {
    CommandInstances(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_,
                                     "instances",
                                     "[<type prefix>] shows the available instances") {
    }
    virtual ~CommandInstances() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confTypesCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() > 1) {
        return false;
      }
      std::cerr << "available instances: " << std::endl;
      for (IdentifiablePtr id : shell->_manager.instances()) {
        ConfigurablePtr c = std::dynamic_pointer_cast<Configurable>(id);
        if (!c) {
          continue;
        }
        if (args.size() == 1 && args[0] != c->className()) {
          continue;
        }

        std::stringstream ss;
        ss << c.get();

        std::cerr << "ptr: " << ULCYAN << ss.str() << RESET << " class: " << std::left
                  << std::setw(70) << c->className() << " name: " << ULCYAN << c->name() << RESET
                  << std::endl;
      }
      return true;
    }
  };

  struct CommandTypes : public ConfigurableShell::CommandBase {
    CommandTypes(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "types", "[<type prefix>] lists available types") {
    }
    virtual ~CommandTypes() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() > 1) {
        return false;
      }

      std::string prefix;
      if (args.size()) {
        prefix = args[0];
      }

      StringVector completions;
      shell->confTypesCompletion(completions, prefix);

      std::cerr << "available types starting with [" << prefix << "]: " << std::endl;
      for (auto& s : completions) {
        std::cerr << s << std::endl;
      }
      return true;
    }
  };

  struct CommandSetName : public ConfigurableShell::CommandBase {
    CommandSetName(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_,
                               "set_name",
                               "<name/ptr> <new_name>,  sets the name of an object ") {
    }
    virtual ~CommandSetName() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() == 1) {
        shell->confPtrCompletion(completions, args[0]);
        return;
      }
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() < 1) {
        return false;
      }
      ConfigurablePtr conf = shell->getConfigurable(args[0]);
      if (!conf) {
        std::cerr << "object [" << args[0] << "] unknown, cannot set name" << std::endl;
        return false;
      }
      if (args.size() > 1) {
        auto it = shell->_manager.namedInstances().find(args[1]);
        if (it != shell->_manager.namedInstances().end() && it->second != conf) {
          std::cerr << "cannot rename, object with the same name exists : " << it->second
                    << std::endl;
        }

        shell->_manager.rename(conf, args[1]);
        std::cerr << "renamed object " << conf << " new name: [" << conf->name() << "]"
                  << std::endl;
      } else {
        shell->_manager.rename(conf, "");
        std::cerr << "erased name for object " << conf << std::endl;
      }
      return true;
    }
  };

  struct CommandShow : public ConfigurableShell::CommandBase {
    CommandShow(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_, "show", "<name/ptr>,  shows the fields of an object ") {
    }
    virtual ~CommandShow() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() != 1) {
        return;
      }
      shell->confPtrCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      ConfigurablePtr conf = shell->getConfigurable(args[0]);
      if (!conf) {
        std::cerr << "object [" << args[0] << "] unknown, cannot show" << std::endl;
        return false;
      }
      IdContext id_context;
      ObjectData odata;
      odata.setPointer("pointer", conf.get());
      conf->serialize(odata, id_context);
      JSONObjectWriter writer;
      writer.writeObject(std::cerr, conf->className(), odata);
      return true;
    }
  };

  struct CommandCreate : public ConfigurableShell::CommandBase {
    CommandCreate(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_, "create", "<class_name>,  creates a new instance") {
    }
    virtual ~CommandCreate() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confTypesCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      std::vector<std::string> types = ConfigurableManager::listTypes();
      bool found                     = std::binary_search(types.begin(), types.end(), args[0]);
      if (!found) {
        std::cerr << "unknown type [" << args[0] << "] unknown, cannot create" << std::endl;
        return false;
      }
      IdentifiablePtr ptr = shell->_manager.create(args[0]);
      std::cerr << "created object of type " << args[0] << "ptr: " << ptr << std::endl;
      return true;
    }
  };

  struct CommandErase : public ConfigurableShell::CommandBase {
    CommandErase(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_, "erase", "<class_name/ptr>, erases an instance") {
    }
    virtual ~CommandErase() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confPtrCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() < 1) {
        return false;
      }
      for (const std::string& s : args) {
        ConfigurablePtr module = shell->getConfigurable(s);
        if (!module) {
          std::cerr << "object [" << FG_CYAN(s) << "] unknown, cannot erase" << std::endl;
          return false;
        }
        shell->_manager.erase(module);
        std::cerr << "destroyed object " << FG_CYAN(s) << std::endl;
      }
      return true;
    }
  };

  struct CommandExec : public ConfigurableShell::CommandBase {
    CommandExec(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_,
                               "exec",
                               "<class_name/ptr> [<args>], executes a module a command") {
    }
    virtual ~CommandExec() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confPtrCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() < 1) {
        return false;
      }
      for (const std::string& s : args) {
        ConfigurablePtr module = shell->getConfigurable(s);
        if (!module) {
          std::cerr << "object [" << FG_CYAN(s) << "] unknown, cannot exec" << std::endl;
          return false;
        }
        std::string response;
        std::vector<std::string> tokens;
        for (size_t i = 1; i < args.size(); ++i) {
          tokens.push_back(args[i]);
        }
        bool ok = module->handleCommand(shell, response, tokens);
        std::cerr << response << std::endl;
        if (ok) {
          return true;
        }
      }
      return true;
    }
  };

  struct CommandSet : public ConfigurableShell::CommandBase {
    CommandSet(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_, "set", "<class_name>, sets a field in a config") {
    }
    virtual ~CommandSet() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() < 1) {
        return;
      }
      if (args.size() == 1) {
        shell->confPtrCompletion(completions, args[0]);
        return;
      }
      ConfigurablePtr conf = shell->getConfigurable(args[0]);
      if (!conf) {
        return;
      }

      if (args.size() == 2) {
        shell->confFieldCompletion(completions, args[1], conf);
        return;
      }
      auto prop_it = conf->properties().find(args[1]);
      if (prop_it == conf->properties().end()) {
        return;
      }
      PropertyBase* prop = prop_it->second;

      if (!prop) {
        return;
      }

      PropertyIdentifiablePtrInterfaceBase* pc =
        dynamic_cast<PropertyIdentifiablePtrInterfaceBase*>(prop);
      if (!pc) {
        return;
      }
      const std::string& last_arg = args[args.size() - 1];

      StringVector candidates;
      shell->confPtrCompletion(candidates, last_arg);
      // we check the class name of each candidate, and if not valid we erase it
      for (const std::string& s : candidates) {
        ConfigurablePtr conf = shell->getConfigurable(s);
        if (pc->canAssign(conf)) {
          completions.push_back(s);
        }
      }
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() < 3) {
        return false;
      }
      ConfigurablePtr conf = shell->getConfigurable(args[0]);
      if (!conf) {
        std::cerr << "target object [" << args[0] << "] unknown, cannot set" << std::endl;
        return false;
      }
      auto prop_it = conf->properties().find(args[1]);
      if (prop_it == conf->properties().end()) {
        std::cerr << "class [" << conf->className() << "] does not have field [" << args[1] << "]"
                  << std::endl;
        return false;
      }
      PropertyIdentifiablePtrVectorInterface* pcv =
        dynamic_cast<PropertyIdentifiablePtrVectorInterface*>(prop_it->second);
      if (pcv) {
        std::vector<ConfigurablePtr> src_modules;
        for (size_t i = 2; i < args.size(); ++i) {
          ConfigurablePtr src_module = shell->getConfigurable(args[i]);
          if (src_module && !pcv->canAssign(src_module)) {
            std::cerr << "incompatible objects" << std::endl;
            return false;
          }
          src_modules.push_back(src_module);
        }
        pcv->assign(src_modules);
        return true;
      }

      PropertyIdentifiablePtrInterface* pc =
        dynamic_cast<PropertyIdentifiablePtrInterface*>(prop_it->second);
      if (pc) {
        if (args[2] == "0") {
          ConfigurablePtr null_module;
          std::cerr << "set successful" << std::endl;
          pc->assign(null_module);
          return true;
        }
        ConfigurablePtr val = shell->getConfigurable(args[2]);
        if (!val) {
          std::cerr << "src object [" << args[2] << "] unknown, cannot set" << std::endl;
          return false;
        }
        if (!pc->assign(val)) {
          std::cerr << "incompatible objects" << std::endl;
          return false;
        }
        std::cerr << "set successful" << std::endl;
        return true;
      }
      std::vector<std::string> tokens;
      for (size_t i = 2; i < args.size(); ++i)
        tokens.push_back(args[i]);
      bool result = prop_it->second->fromTokens(tokens);
      if (!result) {
        std::cerr << "parse error on field" << std::endl;
        return false;
      }
      std::cerr << "set successful" << std::endl;
      return true;
    }
  };

  struct CommandRunRunner {
    static constexpr size_t STACK_SIZE = (8192 << 8); // 8MB

    CommandRunRunner(MessageSourceBasePtr source_, MessageSinkBasePtr sink_) :
      source(source_),
      sink(sink_) {
      root     = source->getRootSource();
      instance = this;
    }
    ~CommandRunRunner() {
      std::cerr << "CommandRunRunner::~CommandRunRunner|job done" << std::endl;
    }

    void retrievePlatformSource() {
      MessagePlatformSinkPtr platform_sink = std::dynamic_pointer_cast<MessagePlatformSink>(sink);
      MessageSourcePlatformPtr platform_source;

      std::shared_ptr<MessageFilterBase> s = std::dynamic_pointer_cast<MessageFilterBase>(source);
      while (s) {
        platform_source = std::dynamic_pointer_cast<MessageSourcePlatform>(s);
        if (platform_source) {
          PlatformPtr platform = platform_source->platform(platform_sink->param_tf_topic.value());
          if (platform && platform->isWellFormed()) {
            platform_sink->setPlatform(platform);
            std::cerr << "CommandRunRunner::retrievePlatformSource|"
                      << FG_GREEN("platform assigned:\n"
                                  << platform_sink->platform())
                      << std::endl;
            instance->requires_platform = false;
          }
          return;
        }
        std::shared_ptr<MessageFilterBase> next =
          std::dynamic_pointer_cast<MessageFilterBase>(s->param_source.value());
        s = next;
      }
    }

    static void startRunner() {
      getcontext(&shell_context);
      runner_context                  = shell_context;
      runner_context.uc_stack.ss_sp   = runner_context_stack;
      runner_context.uc_stack.ss_size = STACK_SIZE;
      runner_context.uc_link          = &shell_context;
      makecontext(&runner_context, runnerThread, 0);
      swapcontext(&shell_context, &runner_context);
    }

    static void resumeRunner() {
      if (instance) {
        // instance->requires_platform = true;
        swapcontext(&shell_context, &runner_context);
      }
    }

    static void resumeShell() {
      if (instance) {
        swapcontext(&runner_context, &shell_context);
      }
    }

    static void runnerThread() {
      BaseSensorMessagePtr msg;
      instance->requires_platform =
        (std::dynamic_pointer_cast<MessagePlatformSink>(instance->sink) != 0);

      while ((msg = instance->source->getMessage())) {
        if (instance->requires_platform) {
          instance->retrievePlatformSource();
        }

        bool step_done = instance->sink->putMessage(msg);
        // DrawableBasePtr
        // drawable=std::dynamic_pointer_cast<DrawableBase>(CommandRunRunner::instance->sink);
        if (step_done) {
          ConfigurableShell::instance->stepCallback(instance->sink);
        }

        // drawable->draw(CommandRunRunner::instance->canvas);
        if (step_mode) {
          resumeShell();
        }
      }
      CommandRunRunner* temp_instance = instance;
      instance                        = 0;
      delete temp_instance;
    }

    static void waitToDie() {
      if (!instance || !instance->root) {
        return;
      }
      instance->root->_running = false;
      delete instance;
      instance = 0;
    }

    static void runnerStop(int v __attribute__((unused))) {
      if (instance) {
        step_mode = true;
      }
    }

    MessageSourceBasePtr source;
    MessageSinkBasePtr sink;
    MessageSourceBase* root;
    bool requires_platform = false;
    static CommandRunRunner* instance; // instance to this object

    static char runner_context_stack[STACK_SIZE];
    static ucontext_t shell_context, runner_context;
    static volatile bool step_mode;
  };

  // cointext shit
  CommandRunRunner* CommandRunRunner::instance = 0;
  volatile bool CommandRunRunner::step_mode    = false;
  char CommandRunRunner::runner_context_stack[STACK_SIZE];
  ucontext_t CommandRunRunner::shell_context;
  ucontext_t CommandRunRunner::runner_context;

  ConfigurableShell::CommandRun::CommandRun(ConfigurableShell* shell_) :
    ConfigurableShell::CommandBase(
      shell_,
      "run",
      "<source> <sink> [--pause], runs a pipeline by pushing all messages from source to sink") {
  }

  bool ConfigurableShell::CommandRun::execute(const std::vector<std::string>& args) {
    if (args.size() < 2 || args.size() > 3) {
      return false;
    }
    ConfigurablePtr source_ = shell->getConfigurable(args[0]);
    if (!source_) {
      std::cerr << "source [" << args[0] << "] unknown, cannot run" << std::endl;
      return false;
    }
    ConfigurablePtr sink_ = shell->getConfigurable(args[1]);
    if (!sink_) {
      std::cerr << "sink [" << args[1] << "] unknown, cannot run" << std::endl;
      return false;
    }
    MessageSourceBasePtr source = std::dynamic_pointer_cast<MessageSourceBase>(source_);
    if (!source) {
      std::cerr << "invalid source cast" << std::endl;
      return false;
    }
    MessageSinkBasePtr sink = std::dynamic_pointer_cast<MessageSinkBase>(sink_);
    if (!sink) {
      std::cerr << "invalid sink cast" << std::endl;
      return false;
    }
    if (CommandRunRunner::instance) {
      std::cerr << "already running" << std::endl;
      std::cerr << "call [kill] to suppress execution" << std::endl;
    }

    preRun();
    new CommandRunRunner(source, sink);
    if (args.size() == 3 && args[2] == "--pause") {
      CommandRunRunner::step_mode = true;
    }
    CommandRunRunner::startRunner();
    // erase signals;
    return true;
  }

  struct CommandKill : public ConfigurableShell::CommandBase {
    CommandKill(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "kill", "kills a running session") {
    }
    virtual ~CommandKill() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size()) {
        return false;
      }

      if (!CommandRunRunner::instance) {
        std::cerr << "no process active, cannot kill" << std::endl;
        return false;
      }
      CommandRunRunner::instance->root->_running = false;
      std::cerr << "waiting for proper termination" << std::endl;
      CommandRunRunner::instance->waitToDie();
      return true;
    }
  };

  struct CommandForeground : public ConfigurableShell::CommandBase {
    CommandForeground(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "fg", "resumes a running process") {
    }
    virtual ~CommandForeground() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size()) {
        return false;
      }

      if (!CommandRunRunner::instance) {
        std::cerr << "no process active, cannot kill" << std::endl;
        return false;
      }
      CommandRunRunner::step_mode = false;
      CommandRunRunner::resumeRunner();
      return true;
    }
  };

  static void completionCallback(const char* line, linenoiseCompletions* completions) {
    if (!ConfigurableShell::instance)
      return;
    StringVector comp;
    ConfigurableShell::instance->confLineCompletions(comp, line);

    for (const std::string& s : comp) {
      linenoiseAddCompletion(completions, s.c_str());
    }
  }

  void ConfigurableShell::clear() {
    std::cerr << "ConfigurableShell::clear|destroying all objects" << std::endl;
    const size_t total_objects = _manager.instances().size();
    size_t k                   = 0;
    while (!_manager.instances().empty()) {
      PropertyContainerIdentifiablePtr c = *_manager.instances().begin();
      if (c) {
        std::cerr << "\tdestroyed object ptr: " << FG_ULRED(c.get()) << std::endl;
        _manager.erase(c);
        ++k;
      }
    }
    std::cerr << "ConfigurableShell::clear|destroyed [" << RED << k << "/" << total_objects << RESET
              << "] objects" << std::endl;
  }

  void ConfigurableShell::CommandBase::completions(StringVector& completions,
                                                   const StringVector& args) {
    completions.clear();
  }

  void ConfigurableShell::run(const StringVector& lines) {
    startCallback();
    if (lines.size()) {
      std::cerr << "ConfigurableShell::run|shell is in batch_mode" << std::endl;
    }

    for (const std::string& s : lines) {
      std::istringstream is(s);
      parseLine(is);
    }

    std::cerr << "ConfigurableShell::run|shell is in interactive_mode" << std::endl;

    while (_run) {
      char* line = linenoise("SRRG> ");
      if (!line) {
        continue;
      }
      std::istringstream is(line);
      parseLine(is);
      if (line[0]) {
        linenoiseHistoryAdd(line);
      }
      free(line);
    }
    terminationCallback();
  }

  void ConfigurableShell::addCommand(CommandBase* cmd) {
    // srrg remove warning
    //    const auto& ret =
    _command_map.insert(std::make_pair(cmd->tag, std::unique_ptr<CommandBase>(cmd)));
  }

  void ConfigurableShell::removeCommand(const std::string& tag_) {
    auto ret = _command_map.find(tag_);
    if (ret != _command_map.end()) {
      _command_map.erase(ret);
    }
  }

  ConfigurablePtr ConfigurableShell::getConfigurable(const std::string& name) {
    // first we seek in the manager named map
    auto map_it = _manager.namedInstances().find(name);
    if (map_it != _manager.namedInstances().end()) {
      return std::dynamic_pointer_cast<Configurable>(map_it->second);
    }
    void* ptr_val;
    std::istringstream is(name.c_str());

    is >> ptr_val;

    for (IdentifiablePtr p : _manager.instances()) {
      if (p.get() == ptr_val) {
        return std::dynamic_pointer_cast<Configurable>(p);
      }
    }
    return 0;
  }

  void ConfigurableShell::confPtrCompletion(StringVector& completions, const std::string& partial) {
    std::ostringstream os;
    for (PropertyContainerIdentifiablePtr c : _manager.instances()) {
      std::ostringstream os;
      os << c;
      std::string ptr_name = os.str();
      auto res             = std::mismatch(partial.begin(), partial.end(), ptr_name.begin());
      if (res.first == partial.end()) {
        completions.push_back(ptr_name);
      }
      if (!c->name().length()) {
        continue;
      }
      auto res2 = std::mismatch(partial.begin(), partial.end(), c->name().begin());
      if (res2.first == partial.end()) {
        completions.push_back(c->name());
      }
    }
  }

  void ConfigurableShell::confFileCompletion(StringVector& completions,
                                             const std::string& prefix_,
                                             bool is_dir,
                                             bool is_file) {
    std::string path;
    std::string suffix;
    std::string prefix;
    if (prefix_.empty() || prefix_[0] != '/') {
      prefix = std::string(getcwd(NULL, 0)) + "/" + prefix;
    }
    if (!prefix_.empty() && prefix_[0] == '/') {
      prefix = prefix_;
    }

    std::size_t pos = prefix.find_last_of('/');
    if (pos != std::string::npos) {
      path   = prefix.substr(0, pos + 1);
      suffix = prefix.substr(pos + 1);
    }
    DIR* dp;
    struct dirent* ep;
    dp = opendir(path.c_str());
    if (dp == NULL)
      return;

    while ((ep = readdir(dp))) {
      std::string entry_path(ep->d_name);
      if (((ep->d_type == DT_DIR) && !is_dir) || ((ep->d_type != DT_DIR) && !is_file))
        continue;

      if (ep->d_type == DT_DIR)
        entry_path += "/";

      if (suffix.empty()) {
        completions.push_back(path + entry_path);
        continue;
      }
      auto res = std::mismatch(suffix.begin(), suffix.end(), entry_path.begin());
      if (res.first == suffix.end()) {
        completions.push_back(path + entry_path);
        continue;
      }
    }
    closedir(dp);
  }

  void ConfigurableShell::confTypesCompletion(StringVector& completions,
                                              const std::string& partial) {
    std::vector<std::string> types = ConfigurableManager::listTypes();
    for (const std::string& t : types) {
      auto res = std::mismatch(partial.begin(), partial.end(), t.begin());
      if (res.first == partial.end()) {
        completions.push_back(t);
        continue;
      }
    }
  }

  void ConfigurableShell::confFieldCompletion(StringVector& completions,
                                              const std::string& partial,
                                              ConfigurablePtr c) {
    for (auto it : c->properties()) {
      const std::string& field_name = it.first;
      auto res = std::mismatch(partial.begin(), partial.end(), field_name.begin());
      if (res.first == partial.end()) {
        completions.push_back(field_name);
        continue;
      }
    }
  }

  void ConfigurableShell::confCommandCompletion(StringVector& completions,
                                                const std::string& partial) {
    for (auto& it : _command_map) {
      const std::string& cmd = it.first;
      auto res               = std::mismatch(partial.begin(), partial.end(), cmd.begin());
      if (res.first == partial.end()) {
        completions.push_back(cmd);
      }
    }
  }

  void ConfigurableShell::confLineCompletions(StringVector& completions, const char* line) {
#define MAX_LINE_LENGTH 10240
    completions.clear();
    // tokenize the line
    StringVector tokens;
    const char* c         = line;
    const int line_length = strlen(line);
    if (line_length > MAX_LINE_LENGTH)
      throw std::runtime_error("shell line buffer overrun");
    char temp_buffer[MAX_LINE_LENGTH];
    char* t = temp_buffer;
    while (*c) {
      *t = *c;
      if (*t == ' ' || !*t) {
        *t = 0;
        if (t != temp_buffer) {
          tokens.push_back(std::string(temp_buffer));
        }
        t = temp_buffer;
      } else {
        ++t;
      }
      ++c;
    }
    if (t != temp_buffer) {
      *t = 0;
      tokens.push_back(temp_buffer);
    }

    // handle last dummy token
    if (!tokens.size() || (line_length && line[line_length - 1] == ' ')) {
      tokens.push_back(std::string());
    }

    // if 1 token we need to complete the commands
    if (tokens.size() == 1) {
      confCommandCompletion(completions, tokens[0]);
      return;
    }

    // we seek for a command
    auto it = _command_map.find(tokens[0]);
    if (it == _command_map.end()) {
      return;
    }

    StringVector args = tokens;

    args.erase(args.begin());

    // ask the command for the proper completions
    // of the __word__

    StringVector word_completions;
    it->second->completions(word_completions, args);

    std::string s_base = tokens[0];
    for (size_t i = 1; i < tokens.size() - 1; ++i) {
      s_base += std::string(" ");
      s_base += tokens[i];
    }

    for (std::string& w : word_completions) {
      completions.push_back(s_base + " " + w);
    }
  }

  void ConfigurableShell::parseLine(std::istream& is) {
    char buffer[1024];
    is.getline(buffer, 1024);
    std::istringstream ss(buffer);
    std::string tag;
    ss >> tag;
    if (!ss) {
      return;
    }
    std::vector<std::string> args;
    while (ss) {
      std::string arg;
      ss >> arg;
      if (ss)
        args.push_back(arg);
    }
    auto cmd_it = _command_map.find(tag);
    if (cmd_it == _command_map.end()) {
      std::cerr << "ConfigurableShell::parseLine|unknown command [" << FG_YELLOW(tag) << "]"
                << std::endl;
      return;
    }
    bool result = cmd_it->second->execute(args);

    if (!result) {
      std::cerr << "ConfigurableShell::parseLine|usage: " << cmd_it->second->help_message
                << std::endl;
    }
    return;
  }

  static struct sigaction new_stop_action, old_stop_action;
  static void ctrlJCallback() {
    if (!CommandRunRunner::instance) {
      return;
    }
    CommandRunRunner::instance->step_mode = true;
    CommandRunRunner::resumeRunner();
  }

  void ConfigurableShell::addCommands() {
    addCommand(new CommandHelp(this));
    addCommand(new CommandPwd(this));
    addCommand(new CommandLs(this));
    addCommand(new CommandCd(this));
    addCommand(new CommandDLConfig(this));
    addCommand(new CommandDLOpen(this));
    addCommand(new CommandOpen(this));
    addCommand(new CommandQuit(this));
    addCommand(new CommandInstances(this));
    addCommand(new CommandTypes(this));
    addCommand(new CommandSetName(this));
    addCommand(new CommandShow(this));
    addCommand(new CommandCreate(this));
    addCommand(new CommandErase(this));
    addCommand(new CommandWrite(this));
    addCommand(new CommandSet(this));
    addCommand(new CommandNames(this));
    addCommand(new CommandClear(this));
    addCommand(new CommandRun(this));
    addCommand(new CommandKill(this));
    addCommand(new CommandExec(this));
    addCommand(new CommandForeground(this));
  }

  ConfigurableShell::ConfigurableShell(ConfigurableManager& manager_) : _manager(manager_) {
    _manager.initFactory();
    std::vector<std::string> types;
    for (auto s : types) {
      std::cerr << s << std::endl;
    }

    instance = this;
    linenoiseSetCompletionCallback(completionCallback);
    linenoiseSetCtrlJCallback(ctrlJCallback);
    sigemptyset(&new_stop_action.sa_mask);
    new_stop_action.sa_flags   = SA_RESTART;
    new_stop_action.sa_handler = CommandRunRunner::runnerStop;
    int sigstop_result         = sigaction(SIGINT, &new_stop_action, &old_stop_action);
    std::cerr << "ConfigurableShell::ConfigurableShell|installed signal handler [" << sigstop_result
              << "]" << std::endl;

    this->addCommands();
  }

  ConfigurableShell::~ConfigurableShell() {
    clear();
    instance = 0;
    linenoiseSetCompletionCallback(0);
    int sigstop_result = sigaction(SIGINT, &old_stop_action, &new_stop_action);
    std::cerr << "ConfigurableShell::~ConfigurableShell|restored signal handler [" << sigstop_result
              << "]" << std::endl;
  }

  void ConfigurableShell::runStatic(const StringVector& lines, std::atomic<bool>* ready_) {
    _ready = ready_;
    instance->run(lines);
  }

  void ConfigurableShell::stepCallback(ConfigurablePtr drawable) {
  }

  void ConfigurableShell::terminationCallback() {
  }

  void ConfigurableShell::startCallback() {
  }

  ConfigurableShell* ConfigurableShell::instance;

} // namespace srrg2_core
