#include "srrg_geometry/epipolar.h"
#include "srrg_geometry/geometry2d.h"
#include "srrg_geometry/geometry3d.h"
#include <srrg_system_utils/shell_colors.h>
#include <srrg_test/test_helper.hpp>

using namespace srrg2_core;
using namespace srrg2_core::epipolar;

void makePoints(Vector3fVector& points,
                size_t num_points,
                float range){
  points.resize(num_points);
  for (size_t i=0; i<num_points; ++i) {
    Vector3f& point=points[i];
    point=Vector3f::Random()*range;
  }
}

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(Epipolar, genpoints) {
  Vector3fVector points;
  makePoints(points, 1000, 10);
}

TEST(Epipolar, projection_triangulation) {
  Vector3fVector points;
  int num_points=1000;
  float world_size=10;
  makePoints(points, num_points, world_size);

  Vector6f p1;
  p1 << 0.1, 0.2, 0.3, 0.1, 0.2, 0.3;
  Isometry3f iso1=geometry3d::v2t(p1);
  Vector3fVector directions1;
  projectPointsOnSphere(directions1, iso1, points);
  
  Vector6f p2;
  p2 <<1, 2, 3, 0.1, 0.2, 0.3;
  Isometry3f iso2=geometry3d::v2t(p2);
  Vector3fVector directions2;
  projectPointsOnSphere(directions2, iso2, points);

  Vector3fVector points_reconstructed;
  std::vector<float> errors;
  triangulateDirections(points_reconstructed, errors,
                        iso1, iso2, directions1, directions2);

  for (size_t i=0; i<points.size(); ++i) {
    ASSERT_LE(errors[i], 1e-2);
  }
  
}

TEST(Epipolar, Essential) {
  Vector6f p1;
  p1 << 1, 10, 12, 0.6, 0.1, 0.2;
  Isometry3f iso=geometry3d::v2t(p1);
  Matrix3f E=iso2essential(iso);
  Isometry3f iso_v[2];
  essential2iso(iso_v, E);
  // four hp
  bool one_good=false;

  
  // std::cerr << "iso1: " << std::endl << iso_v[0].matrix() << std::endl;
  // std::cerr << "iso2: " << std::endl << iso_v[1].matrix() << std::endl;
  
  std::cerr << "iso: " << std::endl << iso.matrix() << std::endl;
  for (int i=0; i<2; ++i){
    //std::cerr << "checking solution[" << i << "]" << std::endl;
    //std::cerr << iso_v[i].matrix() << std::endl;
    // bad rotation below
    if ( (iso.linear()-iso_v[i].linear()).norm()> 1e-3) {
      //std::cerr << "bad rotation" << std::endl;
      continue;
    }
    float coeff_0=iso.translation()(0)/iso_v[i].translation()(0);
    //std::cerr << "coeff_0: " << coeff_0 << std::endl;
    for (int c=0; c<3; ++c) {
      float coeff=iso.translation()(c)/iso_v[i].translation()(c);
      //std::cerr << "coeff [ " << c << "]: " << coeff << std::endl;
      
      if (fabs(coeff-coeff_0)>1e-4)
        continue;

    }
    one_good=true;
    //std::cerr << iso_v[i].matrix() << " " << iso.matrix()<< std::endl;
  }
  ASSERT_EQ(one_good,true);
}


TEST(Epipolar, eight_point) {
  Vector3fVector points;
  int num_points=8;
  float world_size=10;
  makePoints(points, num_points, world_size);

  Isometry3f iso1=Isometry3f::Identity();
  Vector3fVector directions1;
  projectPointsOnSphere(directions1, iso1, points);
  
  Vector6f p2;
  p2 <<1, 2, 3, 0.3, 0.2, 0.1;
  Isometry3f iso2=geometry3d::v2t(p2);
  Vector3fVector directions2;
  projectPointsOnSphere(directions2, iso2, points);

  Matrix3f E=iso2essential(iso2);
  std::cerr << "Essential (from isometry) " << std::endl;
  std::cerr << E << std::endl;
  Matrix3f E_est;
  
  eightPointEstimate(E_est, directions1, directions2);

  std::cerr << "Essential (reconstructed) " << std::endl;
  std::cerr << E_est << std::endl;

  std::cerr << "Ratio (reconstructed) " << std::endl;
  Matrix3f ratio;
  ratio.array()=E.array()/E_est.array();
  std::cerr << ratio << std::endl;
  float r0=ratio(0,0);
  for (int r=0; r<3; ++r) {
    for (int c=0; c<3; ++c) {
      ASSERT_LT(fabs(ratio(r,c)-r0), 1e-2);
    }
  }
}

TEST(Epipolar, get_transform) {
  Vector3fVector points;
  int num_points=8;
  float world_size=10;
  makePoints(points, num_points, world_size);

  Isometry3f iso1=Isometry3f::Identity();
  Vector3fVector directions1;
  projectPointsOnSphere(directions1, iso1, points);
  
  Vector6f p2;
  p2 <<3, 3, 3, 0.1, 0.2, 0.3;
  Isometry3f iso2=geometry3d::v2t(p2);
  Vector3fVector directions2;
  projectPointsOnSphere(directions2, iso2, points);


  std::cerr << "iso: " << std::endl;
  std::cerr << iso2.matrix() << std::endl;

  Eigen::Isometry3f est_iso;
  estimateTransformFromDirections(est_iso, directions1, directions2);
  std::cerr << "est_iso: " << std::endl;
  std::cerr << est_iso.matrix();

  ASSERT_LT((est_iso.linear()-iso2.linear()).norm(), 1e-2);
  Vector3f r=est_iso.translation().array()/iso2.translation().array();
  float r0=r(0);
  for (int i=0; i<3; ++i) {
    ASSERT_LT(fabs(r(i)-r0),1e-2);
  }
}

TEST(Epipolar, ransac_eight_points) {
  Vector3fVector points;
  int num_points=1000;
  float world_size=10;
  float outlier_ratio=0.4;
  int num_rounds = 100;
  float inlier_threshold=0.02;
  size_t num_outliers=num_points*outlier_ratio;

  makePoints(points, num_points, world_size);

  Isometry3f iso1=Isometry3f::Identity();
  Vector3fVector directions1;
  projectPointsOnSphere(directions1, iso1, points);
  
  Vector6f p2;
  p2 <<3, 3, 3, 0.1, 0.2, 0.3;
  Isometry3f iso2=geometry3d::v2t(p2);
  Vector3fVector directions2;
  projectPointsOnSphere(directions2, iso2, points);

  for (size_t i=0; i<num_outliers; ++i) {
    directions2[i]=Vector3f::Random();
    directions2[i].normalize();
  }
  
  Matrix3f E=iso2essential(iso2);
  std::cerr << "Essential (from isometry) " << std::endl;
  std::cerr << E << std::endl;
  Matrix3f E_est;

  std::vector<float> errors;
  int num_inliers = estimateEssentialRANSACEightPts(E_est,
                                                    errors,
                                                    directions1,
                                                    directions2,
                                                    inlier_threshold,
                                                    num_rounds);

  std::cerr << "Essential (reconstructed) " << std::endl;
  std::cerr << E_est << std::endl;
  std::cerr << "num_inliers: " << num_inliers << std::endl;

  std::cerr << "Ratio (reconstructed) " << std::endl;
  Matrix3f ratio;
  ratio.array()=E.array()/E_est.array();
  std::cerr << ratio << std::endl;
  float r0=ratio(0,0);
  for (int r=0; r<3; ++r) {
    for (int c=0; c<3; ++c) {
      ASSERT_LT(fabs(ratio(r,c)-r0), 1e-1);
    }
  }
}
/*
TEST(Epipolar, rotation) {
  Vector3fVector points;
  int num_points=2;
  float world_size=10;

  makePoints(points, num_points, world_size);

  Isometry3f iso1=Isometry3f::Identity();
  Vector3fVector directions1;
  projectPointsOnSphere(directions1, iso1, points);
  
  Vector6f p2;
  p2 <<0, 0, 0, 0, 0, 0.3;
  Isometry3f iso2=geometry3d::v2t(p2);
  Vector3fVector directions2;
  projectPointsOnSphere(directions2, iso2, points);

  Eigen::AngleAxisf aa;
  aa.fromRotationMatrix(iso2.linear());
  std::cerr<< "axis (orig): " << aa.axis().transpose() << std::endl;
  std::cerr<< "angle (orig): " << aa.angle() << std::endl;
  
  Isometry3f iso_est;
  iso_est.setIdentity();

  Matrix3f R;
  float error=estimateRotationTwoPts(R, directions1, directions2);
  iso_est.linear()=R;
  std::cerr << "error: " << error << std::endl;
  std::cerr << "iso, orig: " << std::endl << iso2.matrix() << std::endl;
  std::cerr << "iso, est: " << std::endl << iso_est.matrix() << std::endl;
}
*/  
