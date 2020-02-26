#pragma once
#include <limits>

#include "srrg_config/configurable.h"
#include "srrg_data_structures/matrix.h"

#include "point_projector.h"

namespace srrg2_core {

  template <typename PointCloudType_>
  struct PointProjectorPolar_
    : public PointProjector_<TRANSFORM_CLASS::PolarProjection,
                             PointCloudType_> {
  public:
    PARAM(PropertyFloat,
          angle_row_min,
          "start row angle  [rad]",
          -M_PI / 2,
          &this->_config_changed);
    PARAM(PropertyFloat,
          angle_col_min,
          "start col angle  [rad]",
          -M_PI,
          &this->_config_changed);
    PARAM(PropertyFloat,
          angle_row_max,
          "end row angle    [rad]",
          M_PI / 2,
          &this->_config_changed);
    PARAM(PropertyFloat,
          angle_col_max,
          "end col angle    [rad]",
          M_PI,
          &this->_config_changed);

    using ThisType = PointProjectorPolar_<PointCloudType_>;
    using BaseType =
      PointProjector_<TRANSFORM_CLASS::PolarProjection, PointCloudType_>;
    using PointCloudType             = typename BaseType::PointCloudType;
    using PointType                  = typename PointCloudType::PointType;
    using IteratorType               = typename PointCloudType::iterator;
    using ConstIteratorType          = typename PointCloudType::const_iterator;
    static constexpr int GeometryDim = PointType::GeometryDim;
    using IsometryType               = Isometry_<float, GeometryDim>;
    using ProjectionType             = Transform_<float, GeometryDim>;
    using ProjectedMatrixType        = typename BaseType::ProjectedMatrixType;
    using TransformedMatrixType      = typename BaseType::ProjectedMatrixType;
    using DepthMatrixType            = Matrix_<float>;
    using IteratorMatrixType         = Matrix_<IteratorType>;
    using ConstIteratorMatrixType    = Matrix_<ConstIteratorType>;
    using IndexMatrixType            = Matrix_<int>;

    PointProjectorPolar_() {
      if (ThisType::GeometryDim == 2) {
        this->param_canvas_rows.setValue(1);
        this->param_canvas_cols.setValue(721);
      } else if (ThisType::GeometryDim == 3) {
        this->param_canvas_rows.setValue(360);
        this->param_canvas_cols.setValue(721);
      }
    }

    void initCameraMatrix() override {
      this->_camera_matrix.setIdentity();
      if (GeometryDim == 2) {
        float sensor_res = (this->param_angle_col_max.value() -
                            this->param_angle_col_min.value()) /
                           this->param_canvas_cols.value();
        this->_camera_matrix << 1.f / sensor_res,
          this->param_canvas_cols.value() / 2.f, 0, 0;
      }
      if (GeometryDim == 3) {
        float sensor_hres = (this->param_angle_col_max.value() -
                             this->param_angle_col_min.value()) /
                            this->param_canvas_cols.value();
        float sensor_vres = (this->param_angle_row_max.value() -
                             this->param_angle_row_min.value()) /
                            this->param_canvas_rows.value();
        this->_camera_matrix << 1.f / sensor_hres, 0,
          this->param_canvas_cols.value() / 2.f, 0, 0, 1.f / sensor_vres,
          this->param_canvas_rows.value() / 2.f, 0;
      }
    }

    void adjustProjectionMatrix(ProjectionType& projection_matrix) override {
      projection_matrix.setIdentity();
      projection_matrix.linear().block(0, 0, GeometryDim - 1, GeometryDim - 1) =
        this->_camera_matrix.block(0, 0, GeometryDim - 1, GeometryDim - 1);
      projection_matrix.translation().head(GeometryDim) =
        this->_camera_matrix.block(0, GeometryDim - 1, GeometryDim, 1);
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_core
