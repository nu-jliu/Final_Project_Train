#ifndef ARM_KINRMATICS__KINEMATICS_HPP___
#define ARM_KINRMATICS__KINEMATICS_HPP___

#include <iostream>
#include <armadillo>

namespace arm_kinematics
{
/// \brief Length of base link
constexpr double L1 = 0.3;

/// \brief Length of shoulder link
constexpr double L2 = 0.2;

/// \brief Length of elbow link
constexpr double L3 = 0.1;

/// \brief Home configuration
const arma::mat44 M{
  {0, 0, 1, 0},
  {1, 0, 0, L3},
  {0, 1, 0, L2 + L3},
  {0, 0, 0, 1}
};

/// \brief Screw axis around space frame
const arma::mat Slist = arma::mat{
  {0, 0, 1, 0, 0, 0},
  {1, 0, 0, 0, L1, 0},
  {1, 0, 0, L1 + L2, 0}
}.t();

/// \brief Screw axis around body frame
const arma::mat Blist = arma::mat{
  {0, 1, 0, 0, 0, -L3},
  {0, 0, 1, -L2, L3, 0},
  {0, 0, 1, 0, L3, 0}
}.t();

/// \brief Joint angles of the arm
struct JointAngles
{
  /// \brief Base joint angle of the arm
  double j0;

  /// \brief Shoulder joint angle of the arm
  double j1;

  /// \brief Elbow joint angle of the arm
  double j2;
};

/// \brief End-effector pose of the arm
struct EEPose
{
  /// \brief X coordinate of the end-effector pose
  double x;

  /// \brief Y coordinate of the end-effector pose
  double y;

  /// \brief Z coordinate of the end-effector pose
  double z;
};

/// \brief Output the JointAngles data structure into stream
/// \param os Output stream object
/// \param joint_angles JointAnlgles object
/// \return Output stream
std::ostream & operator<<(std::ostream & os, const JointAngles & joint_angles);

/// \brief Output the JointAngles data structure into stream
/// \param os Output stream object
/// \param ee_pose EEPose object
/// \return Output stream
std::ostream & operator<<(std::ostream & os, const EEPose & ee_pose);

/// \brief
/// \param joints
/// \return
EEPose compute_fk(JointAngles joints);

/// \brief
/// \param pose
/// \return
JointAngles compute_ik(EEPose pose);
}

#endif
