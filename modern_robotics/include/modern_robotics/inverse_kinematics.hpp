#ifndef MODERN_ROBOTICS__INVERSE_KINEMATICS_HPP___
#define MODERN_ROBOTICS__INVERSE_KINEMATICS_HPP___

#include <tuple>
#include <armadillo>

namespace mr
{
/// \brief Compute the Vb vector
/// \param Blist The joint screw axes in the end-effector frame when the
///              manipulator is at the home position, in the format of a
///              matrix with axes as the columns
/// \param M The home configuration of the end-effector
/// \param T The desired end-effector configuration Tsd
/// \param thetalist An initial guess of joint angles that are close to
///                  satisfying Tsd
/// \return The body twist vector
const arma::vec6 compute_Vb(
  const arma::mat & Blist,
  const arma::mat44 & M,
  const arma::mat44 & T,
  const arma::colvec & thetalist
);

/// \brief Compute the Vs vector
/// \param Tsb The transform between the space frame to body frame
/// \param T The desired end-effector configuration Tsd
/// \return The space twist vector
const arma::vec6 compute_Vs(
  const arma::mat44 & Tsb,
  const arma::mat44 & T
);

/// \brief Computes inverse kinematics in the body frame for an open chain robot
/// \param Blist The joint screw axes in the end-effector frame when the
///              manipulator is at the home position, in the format of a
///              matrix with axes as the columns
/// \param M The home configuration of the end-effector
/// \param T The desired end-effector configuration Tsd
/// \param thetalist0 An initial guess of joint angles that are close to
///                   satisfying Tsd
/// \param emog A small positive tolerance on the end-effector orientation
///             error. The returned joint angles must give an end-effector
///             orientation error less than eomg
/// \param ev A small positive tolerance on the end-effector linear position
///           error. The returned joint angles must give an end-effector
///           position error less than ev
/// \return thetalist: Joint angles that achieve T within the specified
///                    tolerances,
/// \return success: A logical value where TRUE means that the function found
///                  a solution and FALSE means that it ran through the set
///                  number of maximum iterations without finding a solution
///                  within the tolerances eomg and ev.
const std::tuple<arma::colvec, bool> IKinBody(
  const arma::mat & Blist,
  const arma::mat44 & M,
  const arma::mat44 & T,
  const arma::colvec & thetalist0,
  const double emog,
  const double ev
);

/// \brief Computes inverse kinematics in the space frame for an open chain robot
/// \param Slist The joint screw axes in the space frame when the
///              manipulator is at the home position, in the format of a
///              matrix with axes as the columns
/// \param M The home configuration of the end-effector
/// \param T The desired end-effector configuration Tsd
/// \param thetalist0 An initial guess of joint angles that are close to
///                   satisfying Tsd
/// \param emog A small positive tolerance on the end-effector orientation
///             error. The returned joint angles must give an end-effector
///             orientation error less than eomg
/// \param ev A small positive tolerance on the end-effector linear position
///           error. The returned joint angles must give an end-effector
///           position error less than ev
/// \return thetalist: Joint angles that achieve T within the specified
///                       tolerances,
/// \return success: A logical value where TRUE means that the function found
///                  a solution and FALSE means that it ran through the set
///                  number of maximum iterations without finding a solution
///                  within the tolerances eomg and ev.
const std::tuple<arma::colvec, bool> IKinSpace(
  const arma::mat & Slist,
  const arma::mat44 & M,
  const arma::mat44 & T,
  const arma::colvec & thetalist0,
  const double emog,
  const double ev
);
}

#endif
