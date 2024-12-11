#include <catch2/catch_all.hpp>
#include "arm_kinematics/kinematics.hpp"

constexpr double tolerance = 1e-8;

TEST_CASE("Test Joint Angle", "[Joint]")
{
  std::stringstream ss1, ss2;
  arm::JointAngles test1{1.2, 0.4, 0.6};
  arm::JointAngles test2{-0.9, 0.3, 0.45};

  ss1 << test1;
  ss2 << test2;

  CHECK(ss1.str() == "[1.2 0.4 0.6]");
  CHECK(ss2.str() == "[-0.9 0.3 0.45]");
}

TEST_CASE("Test End-Effector Pose", "[EEPose]") {
  std::stringstream ss1, ss2;
  arm::EEPose test1{1.2, 3.2, 4.2};
  arm::EEPose test2{0.8, 1.2, 3.3};

  ss1 << test1;
  ss2 << test2;

  CHECK(ss1.str() == "[1.2 3.2 4.2]");
  CHECK(ss2.str() == "[0.8 1.2 3.3]");
}
