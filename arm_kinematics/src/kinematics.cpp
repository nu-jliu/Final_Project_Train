#include <armadillo>

#include "modern_robotics/forward_kinematics.hpp"
#include "modern_robotics/inverse_kinematics.hpp"

#include "arm_kinematics/kinematics.hpp"


namespace arm
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

double normalize_angle(const double rad)
{
  if ((rad > -M_PI) && (rad <= M_PI)) {
    return rad;
  } else if (rad > M_PI) {
    return normalize_angle(rad - 2 * M_PI);
  } else {
    return normalize_angle(rad + 2 * M_PI);
  }
}

const EEPose compute_fk(const JointAngles & joints)
{
  const arma::colvec5 thetalist{joints.j0, joints.j1, joints.j2, 0.0, 0.0};
  const arma::mat44 T = mr::FKinBody(M, Blist, thetalist);

  return {T.at(0, 3), T.at(1, 3), T.at(2, 3)};
}

const std::tuple<JointAngles, bool> compute_ik(const EEPose & pose, const JointAngles & joints)
{
  const arma::colvec5 thetalist0{joints.j0, joints.j1, joints.j2, 0.0, 0.0};
  const arma::mat44 T{
    {0, 1, 0, pose.x},
    {1, 0, 0, pose.y},
    {0, 0, -1, pose.z},
    {0, 0, 0, 1}
  };
  const auto emog = 1e-2;
  const auto ev = 1e-4;
  const auto result = mr::IKinBody(Blist, M, T, thetalist0, emog, ev);

  const arma::colvec thetalist = std::get<0>(result);
  const bool success = std::get<1>(result);

  // if (success) {
  // std::cout << "success" << std::endl;
  return {{
    normalize_angle(thetalist.at(0)),
    normalize_angle(thetalist.at(1)),
    normalize_angle(thetalist.at(2))
  }, success};
  // } else {
  //   std::cout << "failed" << std::endl;
  //   return {{0, 0, 0}, success};
  // }
}
}
