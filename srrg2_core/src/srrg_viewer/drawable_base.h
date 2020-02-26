#pragma once
#include "viewer_canvas.h"

namespace srrg2_core {

  //! @brief base derivable class that represent an object that could be "drawn"
  //!        using OpenGL and OpenCV
  class DrawableBase {
  public:
    virtual void draw(ViewerCanvasPtr gl_canvas_) const {
      // ds optional overriding
    }
    // ia now disabled, everything goes trough the same canvas - TODO rethink
    // virtual void draw(cv::Mat& cv_canvas_) const {
    //   // ds optional overriding
    // }
    virtual ~DrawableBase();
  };

  using DrawableBasePtr = std::shared_ptr<DrawableBase>;

} // namespace srrg2_core
