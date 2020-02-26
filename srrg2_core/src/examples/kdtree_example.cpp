#include <iostream>

#include <stdio.h>
#include <sys/time.h>

#include <srrg_data_structures/kd_tree.hpp>
#include <srrg_system_utils/system_utils.h>

#define EXAMPLE_LOG std::cerr << "srrg2_core::examples::kdtree_example| "

using namespace std;
using namespace srrg2_core;

int main(int argc, char** argv) {
  const int dim = 3;
  size_t num_points = 10000;
  size_t num_queries = 10000;
  float range = 10;

  KDTree<float, dim>::VectorTDVector kd_tree_points(num_points);
  KDTree<float, dim>::VectorTDVector kd_tree_queries(num_queries);

  EXAMPLE_LOG << "Generating " << num_points << " points... ";
  for(size_t i = 0; i < num_points; ++i) {
    for(int k = 0; k < dim; ++k) {
      float rnd_number = range * (drand48() - 0.5);
      kd_tree_points[i](k) = rnd_number;
    }
  }  
  std::cerr << "done" << endl;
  
  EXAMPLE_LOG << "Generating " << num_queries << " queries... ";
  for(size_t i = 0; i < num_queries; ++i) {
    for(int k = 0; k < dim; ++k) {
      float rnd_number = range * (drand48() - 0.5);
      kd_tree_queries[i](k) = rnd_number;
    }
  } 
  std::cerr << "done" << endl;

  // construct the kd_tree
  float leaf_range = 0.1;
  double t_start = getTime();
  KDTree<float, dim>* kd_tree = new KDTree<float, dim>(kd_tree_points, leaf_range);
  double t_end = getTime();
  EXAMPLE_LOG << "SRRG Tree Creation Time: " << (t_end - t_start) * 1000 << " ms" << endl;
  
  kd_tree->sanityCheck();

  // perform queries
  int found = 0;
  double max_distance = 0.5;
  t_start = getTime();
  for(size_t i = 0; i < num_queries; ++i) {
    KDTree<float, dim>::VectorTD query_point = kd_tree_queries[i];
    KDTree<float, dim>::VectorTD answer;
    int index;
    float approx_distance = kd_tree->findNeighbor(answer, index, query_point, max_distance);
    if (approx_distance > 0) {
      found++;
    }
  }
  t_end = getTime();
  EXAMPLE_LOG << "SRRG Total Search Time : " << (t_end - t_start) * 1000 << " ms" << endl;
  EXAMPLE_LOG << "SRRG KDTree search returned " << found << " successful results over "
       << num_queries << " random queries" << endl;
    
  delete kd_tree;
  
  return 0;
}
