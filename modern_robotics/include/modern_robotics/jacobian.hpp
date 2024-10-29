#ifndef MODERN_ROBOTICS__JACOBIAN_HPP___
#define MODERN_ROBOTICS__JACOBIAN_HPP___

#include <armadillo>

namespace mr
{
/// \brief Computes the body Jacobian for an open chain robot
/// \param Blist The joint screw axes in the end-effector frame when the
///              manipulator is at the home position, in the format of a
///              matrix with axes as the columns
/// \param thetalist A list of joint coordinates
/// \return The body Jacobian corresponding to the inputs (6xn real numbers)
const arma::mat JacobianBody(const arma::mat & Blist, const arma::colvec & thetalist);

/// \brief Computes the space Jacobian for an open chain robot
/// \param Slist The joint screw axes in the space frame when the
///              manipulator is at the home position, in the format of a
///              matrix with axes as the columns
/// \param thetalist A list of joint coordinates
/// \return The space Jacobian cooresponding to the inputs (6xn real numbers)
const arma::mat JacobianSpace(const arma::mat & Slist, const arma::colvec & thetalist);
} /// namespace mr

#endif /// MODERN_ROBOTICS__JACOBIAN_HPP___
