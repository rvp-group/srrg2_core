#include "srrg_geometry/geometry2d.h"
#include "srrg_geometry/geometry3d.h"
#include <srrg_system_utils/shell_colors.h>
#include <srrg_test/test_helper.hpp>

using namespace srrg2_core;
using namespace srrg2_test;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(Similiarity, Sim3f) {
  // ldg don't change any values here, cause there are hardcoded
  // ldg multiplication in order to test operators
  // ldg similiarity values
  const float t_x     = 1.f;
  const float t_y     = 3.f;
  const float t_z     = 5.f;
  const float alpha_x = M_PI / 2;
  const float alpha_y = M_PI / 4;
  const float alpha_z = -M_PI / 2;
  const float scale   = 2.f;

  // ldg constructing a similiarity, rotation, translation, scale
  Similiarity3f sim;
  sim.translation()    = Vector3f(t_x, t_y, t_z);
  sim.inverseScaling() = 1.f / scale;
  const Matrix3f rotation(geometry3d::a2r(Vector3f(alpha_x, alpha_y, alpha_z)));
  sim.linear()                = rotation;
  const Matrix3f& linear_part = sim.linear();

  // ldg asserting getters and setters
  ASSERT_FLOAT_EQ(sim.translation()(0), t_x);
  ASSERT_FLOAT_EQ(sim.translation()(1), t_y);
  ASSERT_FLOAT_EQ(sim.translation()(2), t_z);
  ASSERT_FLOAT_EQ(geometry3d::r2a(linear_part)(0), geometry3d::r2a(rotation)(0));
  ASSERT_FLOAT_EQ(geometry3d::r2a(linear_part)(1), geometry3d::r2a(rotation)(1));
  ASSERT_FLOAT_EQ(geometry3d::r2a(linear_part)(2), geometry3d::r2a(rotation)(2));
  ASSERT_FLOAT_EQ(sim.inverseScaling(), 1.f / scale);

  // ldg asserting = operator
  Similiarity3f new_sim = sim;
  ASSERT_FLOAT_EQ(sim.translation()(0), new_sim.translation()(0));
  ASSERT_FLOAT_EQ(sim.translation()(1), new_sim.translation()(1));
  ASSERT_FLOAT_EQ(sim.translation()(2), new_sim.translation()(2));
  const Matrix3f& sim_rotation     = sim.linear();
  const Matrix3f& new_sim_rotation = new_sim.linear();
  ASSERT_FLOAT_EQ(geometry3d::r2a(sim_rotation)(0), geometry3d::r2a(new_sim_rotation)(0));
  ASSERT_FLOAT_EQ(geometry3d::r2a(sim_rotation)(1), geometry3d::r2a(new_sim_rotation)(1));
  ASSERT_FLOAT_EQ(geometry3d::r2a(sim_rotation)(2), geometry3d::r2a(new_sim_rotation)(2));
  ASSERT_FLOAT_EQ(sim.inverseScaling(), new_sim.inverseScaling());

  // ldg asserting direct initialization
  Similiarity3f direct_sim(sim);
  ASSERT_FLOAT_EQ(sim.translation()(0), direct_sim.translation()(0));
  ASSERT_FLOAT_EQ(sim.translation()(1), direct_sim.translation()(1));
  ASSERT_FLOAT_EQ(sim.translation()(2), direct_sim.translation()(2));
  const Matrix3f& direct_sim_rotation = direct_sim.linear();
  ASSERT_FLOAT_EQ(geometry3d::r2a(sim_rotation)(0), geometry3d::r2a(direct_sim_rotation)(0));
  ASSERT_FLOAT_EQ(geometry3d::r2a(sim_rotation)(1), geometry3d::r2a(direct_sim_rotation)(1));
  ASSERT_FLOAT_EQ(geometry3d::r2a(sim_rotation)(2), geometry3d::r2a(direct_sim_rotation)(2));
  ASSERT_FLOAT_EQ(sim.inverseScaling(), direct_sim.inverseScaling());

  // ldg asserting operator * with similiarity
  Similiarity3f identity = new_sim.inverse() * sim;
  ASSERT_FLOAT_EQ(identity.matrix()(0, 0), 1.f);
  ASSERT_FLOAT_EQ(identity.matrix()(1, 1), 1.f);
  ASSERT_FLOAT_EQ(identity.matrix()(2, 2), 1.f);
  ASSERT_FLOAT_EQ(identity.matrix()(3, 3), 1.f);

  ASSERT_FLOAT_EQ(identity.matrix().col(0).sum(), 1.f);
  ASSERT_FLOAT_EQ(identity.matrix().col(1).sum(), 1.f);
  ASSERT_FLOAT_EQ(identity.matrix().col(2).sum(), 1.f);
  ASSERT_FLOAT_EQ(identity.matrix().col(3).sum(), 1.f);

  // ldg asserting operator * between two different similiarities
  const float s_t_x     = 4.f;
  const float s_t_y     = 1.f;
  const float s_t_z     = 2.f;
  const float s_alpha_x = -M_PI / 2;
  const float s_alpha_y = -M_PI / 4;
  const float s_alpha_z = M_PI / 2;
  const float s_scale   = 1.2f;

  Similiarity3f s_sim;
  s_sim.translation()    = Vector3f(s_t_x, s_t_y, s_t_z);
  s_sim.inverseScaling() = 1.f / s_scale;
  const Matrix3f s_rotation(geometry3d::a2r(Vector3f(s_alpha_x, s_alpha_y, s_alpha_z)));
  s_sim.linear()                = s_rotation;
  const Matrix3f& s_linear_part = s_sim.linear();

  // result calculated in octave
  Similiarity3f sim_result;
  Matrix3f rot_result;
  rot_result << 0.707107f, -0.5f, 0.5f, -0.707107f, -0.5f, 0.5f, 0.f, -0.707107f, -0.707107f;
  sim_result.linear()         = rot_result;
  sim_result.translation()    = Vector3f(-3.65685f, 1.91421f, 0.f);
  sim_result.inverseScaling() = 0.416667f;

  // roundoff error porcodio, copuied from octave
  ASSERT_LE((s_sim * sim).matrix().col(0).sum() - sim_result.matrix().col(0).sum(), 7e-8f);
  ASSERT_LE((s_sim * sim).matrix().col(1).sum() - sim_result.matrix().col(1).sum(), 4e-7f);
  ASSERT_LE((s_sim * sim).matrix().col(2).sum() - sim_result.matrix().col(2).sum(), 4e-8f);
  ASSERT_LE((s_sim * sim).matrix().col(3).sum() - sim_result.matrix().col(3).sum(), 4e-8f);
  ASSERT_LE((s_sim * sim).matrix().determinant() - sim_result.matrix().determinant(), 4e-8f);

  // ldg asserting operator *= between two matrices
  s_sim *= sim;
  ASSERT_LE(s_sim.matrix().col(0).sum() - sim_result.matrix().col(0).sum(), 7e-8f);
  ASSERT_LE(s_sim.matrix().col(1).sum() - sim_result.matrix().col(1).sum(), 4e-7f);
  ASSERT_LE(s_sim.matrix().col(2).sum() - sim_result.matrix().col(2).sum(), 4e-8f);
  ASSERT_LE(s_sim.matrix().col(3).sum() - sim_result.matrix().col(3).sum(), 4e-8f);
  ASSERT_LE(s_sim.matrix().determinant() - sim_result.matrix().determinant(), 4e-8f);

  // ldg asserting operator * between similiarity and vector
  const Vector3f& res_vec = identity * Vector3f(t_x, t_y, t_z);
  ASSERT_FLOAT_EQ(res_vec(0), t_x);
  ASSERT_FLOAT_EQ(res_vec(1), t_y);
  ASSERT_FLOAT_EQ(res_vec(2), t_z);

  // result vector calculated in octave
  const Vector3f vec_result(-2.7088318f, 2.9147172f, -16.82498);
  const Vector3f& sv_result = s_sim * Vector3f(2.f, 3.f, 5.f);
  ASSERT_FLOAT_EQ(sv_result(0), vec_result(0));
  ASSERT_FLOAT_EQ(sv_result(1), vec_result(1));
  ASSERT_FLOAT_EQ(sv_result(2), vec_result(2));

  // ldg asserting operator s2v and v2s with angles in quaternions representation
  Vector7f vec_sim = geometry3d::s2v(sim);
  Similiarity3f v2s_sim;
  v2s_sim = geometry3d::v2s(vec_sim);
  ASSERT_LE(v2s_sim.matrix().col(0).sum() - sim.matrix().col(0).sum(), 7e-8f);
  ASSERT_LE(v2s_sim.matrix().col(1).sum() - sim.matrix().col(1).sum(), 4e-7f);
  ASSERT_LE(v2s_sim.matrix().col(2).sum() - sim.matrix().col(2).sum(), 4e-8f);
  ASSERT_LE(v2s_sim.matrix().col(3).sum() - sim.matrix().col(3).sum(), 4e-8f);
  ASSERT_LE(v2s_sim.matrix().determinant() - sim.matrix().determinant(), 4e-8f);

  // ldg asserting operator s2v and v2s with angles in quaternions representation
  Vector7f tas_sim = geometry3d::s2tas(sim);
  Similiarity3f tas2s_sim;
  tas2s_sim = geometry3d::tas2s(tas_sim);
  ASSERT_LE(tas2s_sim.matrix().col(0).sum() - sim.matrix().col(0).sum(), 7e-8f);
  ASSERT_LE(tas2s_sim.matrix().col(1).sum() - sim.matrix().col(1).sum(), 4e-7f);
  ASSERT_LE(tas2s_sim.matrix().col(2).sum() - sim.matrix().col(2).sum(), 4e-8f);
  ASSERT_LE(tas2s_sim.matrix().col(3).sum() - sim.matrix().col(3).sum(), 4e-8f);
  ASSERT_LE(tas2s_sim.matrix().determinant() - sim.matrix().determinant(), 4e-8f);
}

TEST(Similiarity, Sim2f) {
  // ldg don't change any values here, cause there are hardcoded
  // ldg multiplication in order to test operators
  // ldg similiarity values
  const float t_x   = 1.f;
  const float t_y   = 3.f;
  const float scale = 2.f;
  const float theta = M_PI / 2;

  // ldg constructing a similiarity, rotation, translation, scale
  Similiarity2f sim;
  sim.translation()    = Vector2f(t_x, t_y);
  sim.inverseScaling() = 1.f / scale;
  const Matrix2f rotation(geometry2d::a2r(theta));
  sim.linear()                = rotation;
  const Matrix2f& linear_part = sim.linear();

  // ldg asserting getters and setters
  ASSERT_FLOAT_EQ(sim.translation()(0), t_x);
  ASSERT_FLOAT_EQ(sim.translation()(1), t_y);
  ASSERT_FLOAT_EQ(geometry2d::r2a(linear_part), geometry2d::r2a(rotation));
  ASSERT_FLOAT_EQ(sim.inverseScaling(), 1.f / scale);

  // ldg asserting equal operator
  Similiarity2f new_sim = sim;
  ASSERT_FLOAT_EQ(sim.translation()(0), new_sim.translation()(0));
  ASSERT_FLOAT_EQ(sim.translation()(1), new_sim.translation()(1));
  const Matrix2f& sim_rotation     = sim.linear();
  const Matrix2f& new_sim_rotation = new_sim.linear();
  ASSERT_EQ(geometry2d::r2a(sim_rotation), geometry2d::r2a(new_sim_rotation));
  ASSERT_FLOAT_EQ(sim.inverseScaling(), new_sim.inverseScaling());

  // ldg asserting direct initialization
  Similiarity2f direct_sim(sim);
  ASSERT_FLOAT_EQ(sim.translation()(0), direct_sim.translation()(0));
  ASSERT_FLOAT_EQ(sim.translation()(1), direct_sim.translation()(1));
  const Matrix2f& direct_sim_rotation = direct_sim.linear();
  ASSERT_EQ(geometry2d::r2a(sim_rotation), geometry2d::r2a(direct_sim_rotation));
  ASSERT_FLOAT_EQ(sim.inverseScaling(), direct_sim.inverseScaling());

  // ldg asserting operator * with similiarity
  Similiarity2f identity = new_sim.inverse() * sim;
  ASSERT_FLOAT_EQ(identity.matrix()(0, 0), 1.f);
  ASSERT_FLOAT_EQ(identity.matrix()(1, 1), 1.f);
  ASSERT_FLOAT_EQ(identity.matrix()(2, 2), 1.f);

  ASSERT_FLOAT_EQ(identity.matrix().col(0).sum(), 1.f);
  ASSERT_FLOAT_EQ(identity.matrix().col(1).sum(), 1.f);
  ASSERT_FLOAT_EQ(identity.matrix().col(2).sum(), 1.f);

  // ldg asserting operator * between similiarity and vector
  const Vector2f& res_vec = identity * Vector2f(t_x, t_y);
  ASSERT_FLOAT_EQ(res_vec(0), t_x);
  ASSERT_FLOAT_EQ(res_vec(1), t_y);

  // ldg asserting operator s2v and v2s with angles in quaternions representation
  Vector4f vec_sim = geometry2d::s2v(sim);
  Similiarity2f v2s_sim;
  v2s_sim = geometry2d::v2s(vec_sim);
  ASSERT_LE(v2s_sim.matrix().col(0).sum() - sim.matrix().col(0).sum(), 7e-8f);
  ASSERT_LE(v2s_sim.matrix().col(1).sum() - sim.matrix().col(1).sum(), 4e-7f);
  ASSERT_LE(v2s_sim.matrix().col(2).sum() - sim.matrix().col(2).sum(), 4e-8f);
  ASSERT_LE(v2s_sim.matrix().determinant() - sim.matrix().determinant(), 4e-8f);
}