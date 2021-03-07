#include "path_matrix_search_base.h"
#include "srrg_pcl/point_types.h"
#include "srrg_property/property.h"
#include "srrg_property/property_vector.h"
#include <stdexcept>

namespace srrg2_core {
  class PathMatrixDijkstraSearch : public PathMatrixSearchBase {
  public:
    PARAM(PropertyFloat,
          max_cost,
          "when to stop the search",
          1e6,
          &this->_parent_map_changed_flag);

    PARAM(PropertyFloat,
          min_distance,
          "min distance from an obstacle",
          3.f,
          &this->_search_param_changed_flag);

    PARAM(PropertyVector_<float>,
          cost_polynomial,
          "cost=p0*distance_to_next + p1*d + p2*d^2 + ... pn*d^n",
          std::vector<float>(),
          &this->_search_param_changed_flag);

    virtual void compute();

    // resets the calculation of the minimum cost map to the values stored in the path map
    void reset();

    // sets the source from where to compute the minimal paths
    // might be more than one
    template <typename PointContainerType_>
    inline int setGoals(PointContainerType_& points, bool add = false) {
      if (_path_map_changed_flag) {
        reset();
      }
      using PointType_   = typename PointContainerType_::value_type;
      int num_good_goals = 0;
      for (std::size_t idx = 0; idx < points.size(); ++idx) {
        const PointType_& p = points[idx];
        Eigen::Vector2i ip(p.coordinates().x(), p.coordinates().y());
        if (!_path_map->inside(ip)) {
          continue;
        }
        PathMatrixCell& cell = _path_map->at(ip);
        if (cell.distance < param_min_distance.value()) {
          continue;
        }
        cell.cost=0;
        cell.parent=&cell;
        _queue.push(PathSearchEntry(0.f, &cell));
        ++num_good_goals;
      }
      _search_param_changed_flag = true;
      return num_good_goals;
    }

    inline float traversalCost(float distance_to_next, float distance_to_obstacle) const;

  protected:
    PathSearchQueue _queue;
    bool _parent_map_changed_flag   = true;
    bool _search_param_changed_flag = true;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using PathMatrixDijkstraSearchPtr = std::shared_ptr<PathMatrixDijkstraSearch>;

} // namespace srrg2_core
