#include "configurable_visual_shell.h"
#include <thread>
namespace srrg2_core {

  ViewerCoreBase* ConfigurableVisualShell::viewer_core = nullptr;
  std::map<std::string, ViewerCanvasPtr> ConfigurableVisualShell::canvases;
  std::map<ViewerCanvasPtr, ConfigurablePtr> ConfigurableVisualShell::canvas_config_map;

  struct CommandRunVisual : public ConfigurableShell::CommandRun {
    CommandRunVisual(ConfigurableShell* shell_) : ConfigurableShell::CommandRun(shell_) {
      help_message += " with visual";
    }

    virtual void preRun() override {
      if (shell->_ready) {
        *(shell->_ready) = true;
      }
      if (ConfigurableVisualShell::viewer_core && ConfigurableVisualShell::canvases.size()) {
        // ia busy waiting that viewer is online
        std::cerr << "ConfigurableVisualShell::startCallback|waiting for viewer to start ... ";
        while (!ConfigurableVisualShell::viewer_core->isRunning()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cerr << "ready" << std::endl;
        // ia draw something on screen (25 times since we have to get the lock)
        std::cerr << "ConfigurableVisualShell::startCallback|Clearing screen... " << std::endl;
        for (const auto& canvas : ConfigurableVisualShell::canvases) {
          canvas.second->putText("waiting...");
          canvas.second->flush();
        }
        // ia ready to go
        std::cerr << "ConfigurableVisualShell::startCallback|ready to go" << std::endl;
      }
    }
  };

  struct CommandListCanvases : public ConfigurableVisualShell::CommandBase {
    CommandListCanvases(ConfigurableVisualShell* shell_) :
      ConfigurableVisualShell::CommandBase(shell_, "ls_canvases", "lists all canvases") {
    }
    virtual ~CommandListCanvases() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() > 0) {
        return false;
      }

      std::cerr << "available canvases: " << std::endl;

      for (const auto& c : ConfigurableVisualShell::canvases) {
        auto cd_it = ConfigurableVisualShell::canvas_config_map.find(c.second);
        if (cd_it != ConfigurableVisualShell::canvas_config_map.end()) {
          std::stringstream ss, ss_conf;
          ss << c.second.get();
          ss_conf << cd_it->second.get();
          auto configurable       = shell->getConfigurable(ss_conf.str());
          std::string config_disp = "";
          if (configurable && configurable->name().length()) {
            config_disp = configurable->name();
          }
          std::cerr << std::left << "canvas : " << ULCYAN << ss.str() << RESET << " " << YELLOW
                    << std::setw(16) << c.first << RESET << " module: " << ULCYAN << ss_conf.str()
                    << RESET << " " << YELLOW << config_disp << RESET << std::endl;
        } else {
          std::cerr << "no module paired with canvas " << c.first << std::endl;
          return false;
        }
      }
      return true;
    }
  };

  struct CommandRemoveCanvas : public ConfigurableVisualShell::CommandBase {
    CommandRemoveCanvas(ConfigurableVisualShell* shell_) :
      ConfigurableVisualShell::CommandBase(shell_, "rm_canvas", "<canvas_name>, remove a canvas") {
    }
    virtual ~CommandRemoveCanvas() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() == 1) {
        if (auto s = dynamic_cast<ConfigurableVisualShell*>(shell)) {
          s->canvasCompletion(completions, args[0]);
        }
      }
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      auto c_it = ConfigurableVisualShell::canvases.find(args[0]);
      if (c_it != ConfigurableVisualShell::canvases.end()) {
        auto cd_it = ConfigurableVisualShell::canvas_config_map.find(c_it->second);
        if (cd_it != ConfigurableVisualShell::canvas_config_map.end()) {
          ConfigurableVisualShell::viewer_core->detachCanvas(args[0]);
          ConfigurableVisualShell::canvas_config_map.erase(cd_it);
          ConfigurableVisualShell::canvases.erase(c_it);
        } else {
          std::cerr << "no module paired with canvas " << c_it->first << std::endl;
          return false;
        }
      } else {
        std::cerr << "no canvas called " << args[0] << std::endl;
        return false;
      }

      return true;
    }
  };

  struct CommandAddCanvas : public ConfigurableVisualShell::CommandBase {
    CommandAddCanvas(ConfigurableVisualShell* shell_) :
      ConfigurableVisualShell::CommandBase(shell_,
                                           "add_canvas",
                                           "<canvas_name> <name/ptr>, add a new canvas") {
    }

    virtual ~CommandAddCanvas() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() == 2) {
        shell->confPtrCompletion(completions, args[1]);
      }
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 2) {
        return false;
      }
      ConfigurablePtr conf = shell->getConfigurable(args[1]);
      if (!conf) {
        std::cerr << "object [" << ULCYAN << args[1] << RESET << "] unknown, cannot set the canvas"
                  << std::endl;
        return false;
      }

      // srrg check if is drawable
      DrawableBasePtr drawable = std::dynamic_pointer_cast<DrawableBase>(conf);
      if (!drawable) {
        std::cerr << "trying to put a canvas to a non-drawable object" << std::endl;
        return false;
      }

      const srrg2_core::ViewerCanvasPtr& canvas =
        ConfigurableVisualShell::viewer_core->getCanvas(args[0]);

      std::cerr << "adding new canvas " << YELLOW << args[0] << RESET << " (" << ULCYAN << canvas
                << RESET << ") for module " << ULCYAN << args[1] << RESET << std::endl;

      auto ret = ConfigurableVisualShell::canvases.insert(std::make_pair(args[0], canvas));
      if (ret.second == false) {
        std::cerr << "canvas with name " << YELLOW << args[0] << RESET
                  << " already exists, aborting" << std::endl;
        return false;
      }

      auto ret2 =
        ConfigurableVisualShell::canvas_config_map.insert(std::make_pair(ret.first->second, conf));
      if (ret2.second == false) {
        std::cerr << "module " << ULCYAN << args[1] << RESET << " has already a canvas with name"
                  << ret2.first->first->name() << std::endl;
        return false;
      }

      return true;
    }
  };

  ConfigurableVisualShell::ConfigurableVisualShell(ConfigurableManager& manager_)
    : ConfigurableShell(manager_) {
    this->addCommands();
  }

  void ConfigurableVisualShell::stepCallback(ConfigurablePtr c) {
    if (!canvases.size()) {
      return;
    }
    for (const auto& dc_elem : canvas_config_map) {
      if (DrawableBasePtr drawable = std::dynamic_pointer_cast<DrawableBase>(dc_elem.second)) {
        drawable->draw(dc_elem.first);
      }
    }
  }

  void ConfigurableVisualShell::terminationCallback() {
    if (viewer_core) {
      std::cerr << "ConfigurableVisualShell::terminationCallback|stopping viewer" << std::endl;
      viewer_core->stop();
      // awaken the canvas with a flush;
      for (const auto& canvas : canvases) {
        canvas.second->flush();
      }
    }
  }

  void ConfigurableVisualShell::startCallback() {
    if (viewer_core && canvases.size()) {
      // ia busy waiting that viewer is online
      std::cerr << "ConfigurableVisualShell::startCallback|waiting for viewer to start ... ";
      while (!viewer_core->isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      // ia draw something on screen (25 times since we have to get the lock)
      std::cerr << "ConfigurableVisualShell::startCallback|Clearing screen... " << std::endl;
      for (const auto& canvas : canvases) {
        canvas.second->putText("waiting...");
        canvas.second->flush();
      }
      // ia ready to go
      std::cerr << "ConfigurableVisualShell::startCallback|ready to go" << std::endl;
    }
  }

  void ConfigurableVisualShell::canvasCompletion(StringVector& completions,
                                                 const std::string& partial) {
    std::ostringstream os;
    for (const auto c : canvases) {
      const std::string& canvas = c.first;
      auto res                  = std::mismatch(partial.begin(), partial.end(), canvas.begin());
      if (res.first == partial.end()) {
        completions.push_back(canvas);
      }
    }
  }

  void ConfigurableVisualShell::addCommands() {
    ConfigurableShell::addCommands();
    addCommand(new CommandAddCanvas(this));
    addCommand(new CommandRemoveCanvas(this));
    addCommand(new CommandListCanvases(this));
    removeCommand("run");
    addCommand(new CommandRunVisual(this));
  }

} // namespace srrg2_core
