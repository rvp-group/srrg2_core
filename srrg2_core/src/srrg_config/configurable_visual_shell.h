#pragma once

#include "configurable_shell.h"
#include "srrg_viewer/drawable_base.h"
#include "srrg_viewer/viewer_core_base.h"

namespace srrg2_core {
  // simple shell with visualization
  struct ConfigurableVisualShell : public ConfigurableShell {
    ConfigurableVisualShell(ConfigurableManager& manager_,
                            bool handle_sigint_=true);
    virtual ~ConfigurableVisualShell() {
    }
    // ia pointer to the canvas (to draw things)
    static std::map<std::string, ViewerCanvasPtr> canvases;
    static std::map<ViewerCanvasPtr, ConfigurablePtr> canvas_config_map;

    // ia RAW pointer to the viewer core (it's instanciated in the main so raw pointer is ok to
    // avoid wrong deletion ordering)
    static ViewerCoreBase* viewer_core;

    //! @brief step callback
    void stepCallback(ConfigurablePtr c) override;

    //! @brief quit callback
    void terminationCallback() override;

    //! @brief start callback
    void startCallback() override;

    //! @brief completion helper for canvases
    void canvasCompletion(StringVector& completions, const std::string& partial);

    virtual void addCommands() override;
  };

} // namespace srrg2_core
