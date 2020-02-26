#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/StdVector>

#ifdef __SRRG_PARALLEL_KDTREE__
#include <omp.h>
#endif

namespace srrg2_core {

  /**
     KDTree: implements a KDTree, it requires a type T
     and a dimension D
  */
  template <class T, size_t D>
  class KDTree {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // typedef for defining a vector variable sized points
    using VectorTD       = Eigen::Matrix<T, D, 1>;
    using VectorTDVector = std::vector<VectorTD, Eigen::aligned_allocator<VectorTD>>;
    enum NodeType { Leaf = 0x0, Middle = 0x1 };

    /**
       TreeNode class. Represents a base class for a node in the search tree.

       if the type is a Middle node
       it represents a splitting plane, and has 2 child nodes
       that refer to the set of points to the two sides of the splitting plane.
       A splitting plane is parameterized as a point on a plane and as a normal
       to the plane.

       If the type is a leaf
       it represents a bucket containing points in a neighborhood
       To avoid copies, the bucket is kept as a range
       _min_index.._max_indexindices and the points
       in the _points and _indices
    */
    class TreeNode {
    public:
      friend class KDTree;
      //! ctor
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      TreeNode(KDTree<T, D>* tree_, int node_num = 0) :
        _tree(tree_),
        _node_num(node_num),
        _node_type(KDTree<T, D>::Leaf) {
        _mean.setZero();
        _normal.setZero();
        _left_child  = 0;
        _right_child = 0;
        _min_index   = -1;
        _max_index   = -1;
        _num_points  = 0;
      }

      TreeNode(KDTree<T, D>* tree_,
               int node_num,
               const VectorTD& mean_,
               const VectorTD& normal_,
               TreeNode* left_child  = 0,
               TreeNode* right_child = 0) :
        _tree(tree_),
        _node_num(node_num),
        _node_type(KDTree<T, D>::Middle) {
        _min_index = _max_index = -1;
        assert(normal_.rows() == D);
        assert(mean_.rows() == D);
        _normal      = normal_;
        _mean        = mean_;
        _num_points  = 0;
        _left_child  = left_child;
        _right_child = right_child;
        if (_left_child) {
          _num_points += _left_child->numPoints();
        }
        if (_right_child) {
          _num_points += _right_child->numPoints();
        }
      }

      //! dtor
      ~TreeNode() {
        if (_left_child) {
          delete _left_child;
          _left_child = 0;
        }
        if (_right_child) {
          delete _right_child;
          _right_child = 0;
        }
      }

      //! function to search for the neighbor
      //! @param answer: the neighbor found
      //! @param query: the point to search
      //! @param maximum distance allowed for a point
      //! @returns the distance of the closest point. -1 if no point found
      //! within range
      T findNeighbor(VectorTD& answer,
                     int& index,
                     const VectorTD& query,
                     const T max_distance) const {
        switch (_node_type) {
          case KDTree<T, D>::Leaf: {
            T d_max                         = std::numeric_limits<T>::max();
            const VectorTDVector& points    = this->_tree->_points;
            const std::vector<int>& indices = this->_tree->_indices;
            for (size_t i = _min_index; i < _max_index; ++i) {
              T d = (points[i] - query).squaredNorm();
              if (d < d_max) {
                answer = points[i];
                index  = indices[i];
                d_max  = d;
              }
            }

            if (d_max > max_distance * max_distance) {
              index = -1;
              return -1;
            }
            return d_max;
          }

          case KDTree<T, D>::Middle: {
            bool is_left    = side(query);
            TreeNode* child = is_left ? _left_child : _right_child;
            if (child) {
              if (child->_node_type != KDTree<T, D>::Middle &&
                  child->_node_type != KDTree<T, D>::Leaf) {
                std::cerr << "KDTree::findNeighbor|ERROR, calling sanity check (" << _tree << ")\n";
                std::cerr << "KDTree::findNeighbor|good queries" << _tree->good_queries
                          << std::endl;
                std::cerr << "KDTree::findNeighbor|result: " << _tree->sanityCheck() << std::endl;
              }
              return child->findNeighbor(answer, index, query, max_distance);
            }
          }
            return -1;
          default:
            std::cerr << "KDTree::findNeighbor|ERROR, sanity check (" << _tree
                      << ") : " << _tree->sanityCheck() << std::endl;
            throw std::runtime_error("KDTree::findNeighbor|ERROR, unknown node type");
        }
      }

      //! function to search for all the points near to the query point
      //! @param answer: the neighbors found
      //! @param query: the point to search
      //! @param maximum distance allowed for a point
      //! @returns void
      void findNeighbors(VectorTDVector& answers,
                         std::vector<int>& indices,
                         const VectorTD& query,
                         const T maximum_squared_distance) const {
        assert(maximum_squared_distance >= 0);
        switch (_node_type) {
          case KDTree<T, D>::Leaf: {
            // ds return all points in this leaf that satisfy the distance
            assert(_max_index >= _min_index);
            const size_t number_of_points = _max_index - _min_index;
            answers.clear();
            answers.reserve(number_of_points);
            indices.clear();
            indices.reserve(number_of_points);
            for (size_t i = _min_index; i < _max_index; ++i) {
              assert(i < _tree->_points.size());
              const T distance = (_tree->_points[i] - query).squaredNorm();
              if (distance < maximum_squared_distance) {
                answers.emplace_back(_tree->_points[i]);
                indices.emplace_back(_tree->_indices[i]);
              }
            }
            return;
          }

          case KDTree<T, D>::Middle: {
            bool is_left    = side(query);
            TreeNode* child = is_left ? _left_child : _right_child;
            if (child) {
              if (child->_node_type != KDTree<T, D>::Middle &&
                  child->_node_type != KDTree<T, D>::Leaf) {
                std::cerr << "KDTree::findNeighbors|ERROR, calling sanity check (" << _tree << ")"
                          << std::endl;
                std::cerr << "KDTree::findNeighbors|good queries" << _tree->good_queries
                          << std::endl;
                std::cerr << "KDTree::findNeighbors|result: " << _tree->sanityCheck() << std::endl;
              }
              child->findNeighbors(answers, indices, query, maximum_squared_distance);
              return;
            }
          }
            return;

          default:
            std::cerr << "KDTree::findNeighbors|ERROR, sanity check (" << _tree
                      << ") : " << _tree->sanityCheck() << std::endl;
            throw std::runtime_error("KDTree::findNeighbors|ERROR, unknown node type");
        }
      }

      size_t numPoints() const {
        return _num_points;
      }

      size_t minIndex() const {
        return _min_index;
      }

      size_t maxIndex() const {
        return _max_index;
      }

      inline void setMinIndex(size_t min_index) {
        _min_index = min_index;
      }

      inline void setMaxIndex(size_t max_index) {
        _max_index = max_index;
      }

      //! mean const accessor
      inline const VectorTD& mean() const {
        return _mean;
      }

      //! normal const accessor
      inline const VectorTD& normal() const {
        return _normal;
      }

      //! mean left accessor
      inline TreeNode* leftChild() const {
        return _left_child;
      }

      //! mean right accessor
      inline TreeNode* rightChild() const {
        return _right_child;
      }

      inline bool side(const VectorTD& query_point) const {
        return _normal.dot(query_point - _mean) < 0;
      }

    protected:
      KDTree<T, D>* _tree;
      int _node_num;
      const KDTree<T, D>::NodeType _node_type;
      size_t _min_index;
      size_t _max_index;
      VectorTD _normal;
      VectorTD _mean;
      size_t _num_points;
      TreeNode* _left_child;
      TreeNode* _right_child;
    };

    //! ctor
    KDTree(const VectorTDVector& points_, T max_leaf_range, size_t min_leaf_points = 20) {
      good_queries = 0;
      _num_nodes   = 0;
      _points      = points_;
      _aux_points  = points_;
      _indices.resize(points_.size());
      _aux_indices.resize(points_.size());
      _min_leaf_points = min_leaf_points;
      for (size_t i = 0; i < _indices.size(); ++i) {
        _indices[i] = i;
      }
      _root = _buildTree(0, points_.size(), max_leaf_range, 0);
    }

    //! dtor
    ~KDTree() {
      if (_root) {
        delete _root;
      }
      _root = 0;
    }

    bool sanityCheck() const {
      std::vector<int> checked_indices(_points.size());
      std::fill(checked_indices.begin(), checked_indices.end(), -1);
      const size_t k = sanityCheck(checked_indices, 0, _root);
      if (k != _points.size()) {
        throw std::runtime_error("KDTree::sanityCheck(void)|ERROR, illegal size reported");
      }
      std::sort(checked_indices.begin(), checked_indices.end(), std::less<int>());
      for (size_t i = 0; i < checked_indices.size(); i++) {
        if (i != static_cast<size_t>(checked_indices[i])) {
          throw std::runtime_error("KDTree::sanityCheck(void)|ERROR, missing indices");
        }
      }
      return true;
    }

    int sanityCheck(std::vector<int>& checked_indices, int k, const TreeNode* node) const {
      if (!node) {
        return k;
      }
      switch (node->_node_type) {
        case KDTree<T, D>::Leaf:
          for (size_t i = node->_min_index; i < node->_max_index; i++) {
            int idx = _indices[i];
            if (checked_indices[k] != -1) {
              throw std::runtime_error("KDTree::sanityCheck|ERROR, writing on an occupied index");
            }
            checked_indices[k] = idx;
            k++;
          }
          return k;
        case KDTree<T, D>::Middle:
          k = sanityCheck(checked_indices, k, node->_left_child);
          k = sanityCheck(checked_indices, k, node->_right_child);
          return k;
        default:
          throw std::runtime_error("KDTree::sanityCheck|ERROR, illegal type index");
      }
    }

    //! num_nodes accessor
    inline size_t numNodes() const {
      return _num_nodes;
    }

    inline size_t numPoints() const {
      return _points.size();
    }

    //! function to search for the neighbor
    //! @param answer: the neighbor found
    //! @param query: the point to search
    //! @param maximum distance allowed for a point
    //! @returns the distance of the closest point. -1 if no point found within
    //! range
    inline T
    findNeighbor(VectorTD& answer, int& index, const VectorTD& query, const T max_distance) const {
      if (!_root) {
        throw std::runtime_error("KDTree::findNeighbor|ERROR, no root node");
      }
      good_queries++;
      return _root->findNeighbor(answer, index, query, max_distance);
    }

    //! function to search for all the points in the same leaf of the query
    //! point
    //! @param answers: the neighbors found
    //! @param indices: the indices of the neighbors found
    //! @param query: the query point
    //! @param max_distance: maximum distance allowed for a point
    //! @returns void

    inline void findNeighbors(VectorTDVector& answers,
                              std::vector<int>& indices,
                              const VectorTD& query,
                              const T max_distance) const {
      if (!_root) {
        throw std::runtime_error("KDTree::findNeighbors|ERROR, no root node");
      }

      good_queries++;
      _root->findNeighbors(answers, indices, query, max_distance);
    }

    inline void printKDTree() {
      _printKDTree(_root);
    }

    mutable int good_queries;

  protected:
    /**
       Partitions a point vector in two vectors, computing the splitting plane
       as the largest eigenvalue of the point covariance
       @param mean: the returned mean of the splitting plane
       @param normal: the normal of the splitting plane
       @param left: the returned left vector of points
       @param right: the returned right vector of points
       @param points: the array of points
       @returns the distance of the farthest point from the plane
    */
    T _splitPoints(VectorTD& mean,
                   VectorTD& normal,
                   size_t& num_left_points,
                   const size_t min_index,
                   const size_t max_index) {
      // if points empty, nothing to do
      if (min_index == max_index) {
        return 0;
      }

      const size_t num_points            = max_index - min_index;
      const T inverse_num_points         = 1.0 / num_points;
      VectorTD sum                       = VectorTD::Zero();
      Eigen::Matrix<T, D, D> squared_sum = Eigen::Matrix<T, D, D>::Zero();
      Eigen::Matrix<T, D, D> covariance  = Eigen::Matrix<T, D, D>::Zero();
      for (size_t i = min_index; i < max_index; ++i) {
        sum += _points[i];
        squared_sum += _points[i] * _points[i].transpose();
      }
      mean       = sum * inverse_num_points;
      covariance = squared_sum * inverse_num_points - mean * mean.transpose();

      // eigenvalue decomposition
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, D, D>> solver;
      solver.compute(covariance, Eigen::ComputeEigenvectors);
      normal = solver.eigenvectors().col(D - 1).normalized();

      // the following var will contain the range of points along the normal
      // vector
      T max_distance_from_plane = 0;

      // run through the points and split them in the left or the right set
      size_t left_index  = min_index;
      size_t right_index = max_index;

      size_t num_left  = 0;
      size_t num_right = 0;
      for (size_t i = min_index; i < max_index; ++i) {
        T distance_from_plane = normal.dot(_points[i] - mean);
        if (fabs(distance_from_plane) > max_distance_from_plane) {
          max_distance_from_plane = fabs(distance_from_plane);
        }

        bool side = distance_from_plane < 0;
        if (side) {
          _aux_points[left_index]  = _points[i];
          _aux_indices[left_index] = _indices[i];
          left_index++;
          num_left++;
        } else {
          right_index--;
          _aux_points[right_index]  = _points[i];
          _aux_indices[right_index] = _indices[i];
          num_right++;
        }
      }
      assert(max_index - min_index == num_right + num_left);
      for (size_t i = min_index; i < max_index; ++i) {
        _points[i]  = _aux_points[i];
        _indices[i] = _aux_indices[i];
      }

      num_left_points = num_left;
      return max_distance_from_plane;
    }

    //! function to build the tree
    //! @param points: the points
    //! @param max_leaf_range: specify the size of the "box" below which a leaf
    //! node is generated returns the root of the search tree
    TreeNode*
    _buildTree(const size_t min_index, const size_t max_index, const T max_leaf_range, int level) {
      const size_t num_points = max_index - min_index;
      if (!num_points) {
        return 0;
      }

      VectorTD mean;
      VectorTD normal;
      size_t num_left_points = 0;

      const T range = _splitPoints(mean, normal, num_left_points, min_index, max_index);
      assert(range >= 0);

      TreeNode* node = 0;
      if (range < max_leaf_range || num_points < _min_leaf_points) {
        node = new TreeNode(this, _num_nodes);
        node->setMinIndex(min_index);
        node->setMaxIndex(max_index);
        node->_num_points = num_points;
      } else {
        TreeNode *left_tree, *right_tree;

#ifdef __SRRG_PARALLEL_KDTREE__
        int num_threads = omp_get_max_threads();
        int split_level = -1;
        if (level > 0)
          split_level = floor(log(num_threads) / log(2));

        if (split_level == level) {
#pragma omp parallel sections
          {
#pragma omp section
            {
              left_tree =
                _buildTree(min_index, min_index + num_left_points, max_leaf_range, level + 1);
            }
#pragma omp section
            {
              right_tree =
                _buildTree(min_index + num_left_points, max_index, max_leaf_range, level + 1);
            }
          }
        } else {
#endif //__SRRG_PARALLEL_KDTREE__

          left_tree = _buildTree(min_index, min_index + num_left_points, max_leaf_range, level + 1);
          right_tree =
            _buildTree(min_index + num_left_points, max_index, max_leaf_range, level + 1);
#ifdef __SRRG_PARALLEL_KDTREE__
        }
#endif
        node = new TreeNode(this, _num_nodes, mean, normal, left_tree, right_tree);
      }
      _num_nodes++;
      return node;
    }

    void _printKDTree(TreeNode* node) {
      switch (node->_node_type) {
        case KDTree<T, D>::Leaf:
          std::cerr << "Leaf: " << std::endl;
          for (size_t i = node->_min_index; i < node->_max_index; ++i) {
            std::cerr << _points[i].transpose() << std::endl;
          }
          std::cerr << std::endl;
          break;
        case KDTree<T, D>::Middle:
          _printKDTree(node->leftChild());
          _printKDTree(node->rightChild());
          break;
      }
    }

    size_t _num_nodes;
    size_t _min_leaf_points;
    TreeNode* _root;
    VectorTDVector _aux_points;
    VectorTDVector _points;
    std::vector<int> _indices;
    std::vector<int> _aux_indices;
  };

} // namespace srrg2_core
