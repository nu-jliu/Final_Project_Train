#include "arm_kinematics/kinematics.hpp"

namespace arm_kinematics
{
std::ostream & operator<<(std::ostream & os, const JointAngles & joint_angles)
{
  const auto base = joint_angles.j0;
  const auto shoulder = joint_angles.j1;
  const auto elbow = joint_angles.j2;

  os << "[" << base << " " << shoulder << " " << elbow << "]";
  return os;
}

std::ostream & operator<<(std::ostream & os, const EEPose & ee_pose)
{
  const auto x = ee_pose.x;
  const auto y = ee_pose.y;
  const auto z = ee_pose.z;

  os << "[" << x << " " << y << " " << z << "]";
  return os;
}
}
