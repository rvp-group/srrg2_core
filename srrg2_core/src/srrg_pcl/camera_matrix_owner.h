#pragma once

#include "srrg_property/property.h"

#include "srrg_config/configurable.h"
#include "srrg_geometry/geometry_defs.h"

namespace srrg2_core {

  template <int GeometryDim_>
  class CameraMatrixOwner_ : public Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int GeometryDim = GeometryDim_;
    using IsometryType               = Isometry_<float, GeometryDim>;
    virtual ~CameraMatrixOwner_() {
    }

    using CameraMatrixType = Eigen::Matrix<float, GeometryDim, GeometryDim>;
    PARAM(PropertyInt, canvas_rows, "rows of the canvas", 1, &_config_changed);
    PARAM(PropertyInt, canvas_cols, "cols of the canvas", 721, &_config_changed);
    inline void setCameraMatrix(const CameraMatrixType& camera_matrix_) {
      _camera_matrix = camera_matrix_;
    }

    inline const CameraMatrixType& cameraMatrix() const {
      return _camera_matrix;
    }

    inline void setCameraPose(const IsometryType& cam_in_world) {
      _camera_in_world = cam_in_world;
      _world_in_camera = cam_in_world.inverse();
    }

    inline const IsometryType& cameraPose() const {
      return _camera_in_world;
    }

  protected:
    CameraMatrixType _camera_matrix = CameraMatrixType::Identity();
    IsometryType _camera_in_world   = IsometryType::Identity();
    IsometryType _world_in_camera   = IsometryType::Identity();

    bool _config_changed = true;
  };
  using CameraMatrixOwner2D    = CameraMatrixOwner_<2>;
  using CameraMatrixOwner3D    = CameraMatrixOwner_<3>;
  using CameraMatrixOwner2DPtr = std::shared_ptr<CameraMatrixOwner2D>;
  using CameraMatrixOwner3DPtr = std::shared_ptr<CameraMatrixOwner3D>;

} // namespace srrg2_core
