#pragma once
#include <srrg_config/configurable.h>
#include <srrg_config/configurable_manager.h>
namespace srrg2_core {
  using StringVector = std::vector<std::string>;

  class ConfigurableShell {
  public:
    ConfigurableShell();
    virtual ~ConfigurableShell();
    struct CommandBase {
      CommandBase(ConfigurableShell* shell_,
                  const std::string& tag_,
                  const std::string& help_message_) :
        shell(shell_),
        tag(tag_),
        help_message(help_message_) {
      }

      ConfigurableShell* shell;
      std::string tag;
      std::string help_message;
      virtual bool execute(const std::vector<std::string>& args) = 0;
      virtual void completions(StringVector& completions, const StringVector& args);
    };
    void run(const StringVector& lines = StringVector());
    static void registerAllInstances();
    static void runStatic(const StringVector& lines = StringVector(),
                          std::atomic<bool>* ready_ = nullptr);

    using CommandBasePtr = std::unique_ptr<CommandBase>;
    using CommandMap     = std::map<std::string, CommandBasePtr>;
    // all commands that match prefix
    void confCommandCompletion(StringVector& completions, const std::string& prefix);

    // all types that match prefix
    void confTypesCompletion(StringVector& completions, const std::string& prefix);

    // all pointers/named objects that match prefix
    void confPtrCompletion(StringVector& completions, const std::string& prefix);

    // all pointers/named objects that match prefix
    void confFileCompletion(StringVector& completions,
                             const std::string& prefix,
                             bool is_dir=true,
                             bool is_file=true);

    // all fields in c that match prefix
    void
    confFieldCompletion(StringVector& completions, const std::string& prefix, ConfigurablePtr c);

    // generates completions for an entire line
    void confLineCompletions(StringVector& completions, const char* line);

    ConfigurablePtr getConfigurable(const std::string& name);
    srrg2_core::ConfigurableManager _manager;
    void addCommand(CommandBase* cmd);
    void removeCommand(const std::string& tag_);
    CommandMap _command_map;
    void parseLine(std::istream& is);
    void clear();

    bool _run = true;
    static std::atomic<bool>* _ready;
    // step_callback, called after each message is processed in run mode
    virtual void stepCallback(ConfigurablePtr drawable);

    // termination callback, called when the shell is quit
    virtual void terminationCallback();

    // start callback, called when the shell is quit
    virtual void startCallback();

    virtual void addCommands();
    static ConfigurableShell* instance;

    struct CommandRun : public ConfigurableShell::CommandBase {
      CommandRun(ConfigurableShell* shell_);
      virtual ~CommandRun() {
      }

      bool execute(const std::vector<std::string>& args) override;

      virtual void preRun() {
      }
    };
  };

} // namespace srrg2_core
