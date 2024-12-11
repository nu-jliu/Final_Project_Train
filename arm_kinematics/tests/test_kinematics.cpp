#include <catch2/catch_all.hpp>

#include "arm_kinematics/kinematics.hpp"

constexpr double tolerance = 1e-6;

TEST_CASE("Testing forward kinematics", "[compute_fk]")
{
  const arm::JointAngles joints1{0.0, 0.0, 0.0};
  const arm::EEPose ee_pose1 = arm::compute_fk(joints1);

  // std::cout << ee_pose1 << std::endl;

  CHECK_THAT(ee_pose1.x, Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(ee_pose1.y, Catch::Matchers::WithinAbs(arm::L3, tolerance));
  CHECK_THAT(ee_pose1.z, Catch::Matchers::WithinAbs(arm::L1 + arm::L2, tolerance));

  const arm::JointAngles joints2{0.0, M_PI_4, 0.0};
  const arm::EEPose ee_pose2 = arm::compute_fk(joints2);

  // std::cout << ee_pose2 << std::endl;

  CHECK_THAT(ee_pose2.x, Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(
    ee_pose2.y,
    Catch::Matchers::WithinAbs(
      arm::L2 / sqrt(2) + arm::L3 / sqrt(2),
      tolerance
    )
  );
  CHECK_THAT(
    ee_pose2.z,
    Catch::Matchers::WithinAbs(
      arm::L1 + arm::L2 / sqrt(2) - arm::L3 / sqrt(2),
      tolerance
    )
  );

  const arm::JointAngles joints3{M_PI_2, 0.0, 0.0};
  const arm::EEPose ee_pose3 = arm::compute_fk(joints3);

  std::cout << ee_pose3 << std::endl;
}

TEST_CASE("Testing inverse kinematics", "[compute_ik]")
{
  const arm::EEPose pose1{
    0.0,
    arm::L2 / sqrt(2) + arm::L3 / sqrt(2),
    arm::L1 + arm::L2 / sqrt(2) - arm::L3 / sqrt(2),
  };
  const arm::EEPose pose2{0.0, arm::L3, arm::L1 + arm::L2};

  const arm::JointAngles joints0{0.0, 0.0, 0.0};

  const auto result1 = arm::compute_ik(pose1, joints0);
  const auto result2 = arm::compute_ik(pose2, joints0);

  const bool success1 = std::get<1>(result1);
  const bool success2 = std::get<1>(result2);

  const arm::JointAngles joints1 = std::get<0>(result1);
  const arm::JointAngles joints2 = std::get<0>(result2);

  std::cout << joints1 << std::endl;
  std::cout << joints2 << std::endl;

  const arm::EEPose ee1 = arm::compute_fk(joints1);
  const arm::EEPose ee2 = arm::compute_fk(joints2);

  CHECK(success1);
  CHECK(success2);

  CHECK_THAT(ee1.x, Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(
    ee1.y,
    Catch::Matchers::WithinAbs(
      arm::L2 / sqrt(2) + arm::L3 / sqrt(2),
      tolerance
    )
  );
  CHECK_THAT(
    ee1.z,
    Catch::Matchers::WithinAbs(
      arm::L1 + arm::L2 / sqrt(2) - arm::L3 / sqrt(2),
      tolerance
    )
  );

  CHECK_THAT(ee2.x, Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(ee2.y, Catch::Matchers::WithinAbs(arm::L3, tolerance));
  CHECK_THAT(ee2.z, Catch::Matchers::WithinAbs(arm::L1 + arm::L2, tolerance));

  const arm::EEPose pose3{-arm::L3, 0.0, arm::L1 + arm::L2};
  const auto result3 = arm::compute_ik(pose3, joints0);
  const arm::JointAngles joints3 = std::get<0>(result3);
  const bool success3 = std::get<1>(result3);

  std::cout << std::boolalpha << success3 << std::endl;
}
