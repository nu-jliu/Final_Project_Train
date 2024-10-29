#ifndef MODERN_ROBOTICS__UTILS_HPP___
#define MODERN_ROBOTICS__UTILS_HPP___

#include <armadillo>

namespace mr
{
/// \brief Converts a homogeneous transformation matrix into a rotation matrix
///        and position vector
/// \param T A homogeneous transformation matrix
/// \return The corresponding rotation matrix, The corresponding position vector.
const std::tuple<const arma::mat33, const arma::colvec3> TransToRp(const arma::mat44 & T);

/// \brief Inverts a homogeneous transformation matrix
/// \param T A homogeneous transformation matrix
/// \return The inverse of T
const arma::mat44 TransInv(const arma::mat44 & T);

/// \brief Vector to so3 matrix
/// \param omg The original vector
/// \return The result so3 matrix
const arma::mat33 VecToso3(const arma::vec3 & omg);

/// \brief Convert from so3 matrux to vector
/// \param so3mat source so3 matrix
/// \return result vector
const arma::vec3 so3ToVec(const arma::mat33 & so3mat);

/// \brief Converts a 3-vector of exponential coordinates for rotation into
///        axis-angle form
/// \param expc3 A 3-vector of exponential coordinates for rotation
/// \return A unit rotation axis and the corresponding rotation angle
const std::tuple<const arma::vec3, double> AxisAng3(const arma::vec3 & expc3);

/// \brief Vector to se3 matrix
/// \param V The original vector
/// \return The result se3 matrix
const arma::mat44 VecTose3(const arma::vec6 & V);

/// \brief Converts an se3 matrix into a spatial velocity vector
/// \param se3mat A 4x4 matrix in se3
/// \return The spatial velocity 6-vector corresponding to se3mat
const arma::vec6 se3ToVec(const arma::mat44 & se3mat);

/// \brief Computes the adjoint representation of a homogeneous transformation
///        matrix
/// \param T A homogeneous transformation matrix
/// \return The 6x6 adjoint representation [AdT] of T
const arma::mat66 Adjoint(const arma::mat44 & T);

/// \brief Computes the matrix exponential of a matrix in so(3)
/// \param so3mat A 3x3 skew-symmetric matrix
/// \return The matrix exponential of so3mat
const arma::mat33 MatrixExp3(const arma::mat33 & so3mat);

/// \brief Computes the matrix logarithm of a rotation matrix
/// \param R A 3x3 rotation matrix
/// \return The matrix logarithm of R
const arma::mat33 MatrixLog3(const arma::mat33 & R);

/// \brief Computes the matrix exponential of an se3 representation of
///        exponential coordinates
/// \param se3mat A matrix in se3
/// \return The matrix exponential of se3mat
const arma::mat44 MatrixExp6(const arma::mat44 & se3mat);

/// \brief Computes the matrix logarithm of a homogeneous transformation matrix
/// \param T A matrix in SE3
/// \return The matrix logarithm of T
const arma::mat44 MatrixLog6(const arma::mat44 & T);
}

#endif /// MODERN_ROBOTICS__UTILS_HPP___
