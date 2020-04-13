// This file describes a challenge of a C++ L1 Padawan. The goal
// of this unit test is to suggest an API and the abstractions
// needed to implement an isometry.

// Consider including other header files if needed.
// Copyright 2020 <Jose Tomas Lorente>


#include <cmath>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include <isometry/isometry.hpp>

namespace ekumen {
namespace math {
namespace test {
namespace {

  GTEST_TEST(Vector3Test, Vector3FullTests) {
    const double kTolerance{1e-12};
    const Vector3 p{1., 2., 3.};
    const Vector3 q{4., 5., 6.};
    Vector3 r{1., 1., 1.};

    EXPECT_TRUE(Vector3::kUnitX == Vector3(1., 0., 0.));
    EXPECT_TRUE(Vector3::kUnitY == Vector3(0., 1., 0.));
    EXPECT_TRUE(Vector3::kUnitZ == Vector3(0., 0., 1.));
    EXPECT_TRUE(Vector3::kUnitX == std::initializer_list<double>({1., 0., 0.}));
    EXPECT_TRUE(Vector3::kUnitY == std::initializer_list<double>({0., 1., 0.}));
    EXPECT_TRUE(Vector3::kUnitZ == std::initializer_list<double>({0., 0., 1.}));
    EXPECT_TRUE(Vector3::kUnitX != std::initializer_list<double>({1., 1., 0.}));
    EXPECT_TRUE(Vector3::kUnitY != std::initializer_list<double>({0., 1., 1.}));
    EXPECT_TRUE(Vector3::kUnitZ != std::initializer_list<double>({1., 0., 1.}));
    EXPECT_TRUE(Vector3::kUnitZ == Vector3::kUnitX.cross(Vector3::kUnitY));
    EXPECT_NEAR(Vector3::kUnitX.dot(Vector3::kUnitZ), 0., kTolerance);
    EXPECT_NEAR(Vector3::kUnitX.dot(Vector3::kUnitY), 0., kTolerance);
    EXPECT_NEAR(Vector3::kUnitY.dot(Vector3::kUnitZ), 0., kTolerance);

    EXPECT_EQ(p + q, std::initializer_list<double>({5., 7., 9.}));
    EXPECT_EQ(p - q, std::initializer_list<double>({-3., -3., -3.}));
    EXPECT_EQ(p * q, std::initializer_list<double>({4., 10., 18.}));
    EXPECT_EQ(p / q, std::initializer_list<double>({.25, .4, .5}));
    EXPECT_EQ(p * 2., Vector3(2., 4., 6.));
    EXPECT_EQ(q / 2., Vector3(2., 2.5, 3.));
    EXPECT_EQ(2 * q, Vector3(8., 10., 12.));

    EXPECT_EQ(p += q, std::initializer_list<double>({5., 7., 9.}));
  }

}  // namespace
}  // namespace test
}  // namespace math
}  // namespace ekumen

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
