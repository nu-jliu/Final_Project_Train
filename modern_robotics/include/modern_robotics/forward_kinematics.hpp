#ifndef MODERN_ROBOTICS__FORWARD_KINEMATICS_HPP___
#define MODERN_ROBOTICS__FORWARD_KINEMATICS_HPP___

#include <armadillo>

namespace mr
{
/// \brief Computes forward kinematics in the body frame for an open chain robot
/// \param M The home configuration (position and orientation) of the end-effector
/// \param Blist The joint screw axes in the end-effector frame when the
///              manipulator is at the home position, in the format of a
///              matrix with axes as the columns
/// \param thetalist A list of joint coordinates
/// \return A homogeneous transformation matrix representing the end-
///         effector frame when the joints are at the specified coordinates
///         (i.t.o Body Frame)
const arma::mat44 FKinBody(
  const arma::mat44 & M, const arma::mat & Blist,
  const std::vector<double> & thetalist
);

/// \brief Computes forward kinematics in the space frame for an open chain robot
/// \param M The home configuration (position and orientation) of the end-
///          effector
/// \param Slist The joint screw axes in the space frame when the
///              manipulator is at the home position, in the format of a
///              matrix with axes as the columns
/// \param thetalist A list of joint coordinates
/// \return A homogeneous transformation matrix representing the end-
///         effector frame when the joints are at the specified coordinates
///         (i.t.o Space Frame)
const arma::mat44 FKinSpace(
  const arma::mat44 & M, const arma::mat & Slist,
  const std::vector<double> & thetalist
);
}

#endif
