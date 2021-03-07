#pragma once
#include <srrg_config/configurable.h>
#include <deque>
#include <queue>
#include "path_matrix.h"

namespace srrg2_core {

  class PathMatrixSearchBase: public Configurable {
  public:
    // input
    inline void setPathMatrix(PathMatrix* path_map_) {
      _path_map = path_map_;
      _path_map_changed_flag = true;
    }

    inline const PathMatrix* pathMap() const {
      return _path_map;
    }

    PathMatrixSearchBase(): Configurable() {
    }

    virtual ~PathMatrixSearchBase() {
    };
  protected:
    PathMatrix* _path_map = 0;
    // flag toggled when changing path map
    bool _path_map_changed_flag = true;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct PathSearchEntry {
    PathSearchEntry(const float cost_ = std::numeric_limits<float>::max(),
        PathMatrixCell* cell_ = 0):
          cost(cost_),
          cell(cell_) {
    }

    inline bool operator<(const PathSearchEntry& e) const {
      return e.cost < cost;
    }

    float cost;
    PathMatrixCell* cell;
  };

  typedef std::priority_queue<PathSearchEntry> PathSearchQueue;

}  // namespace srrg2_core
