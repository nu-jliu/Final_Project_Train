#ifndef MODERN_ROBOTICS__HELPER_HPP___
#define MODERN_ROBOTICS__HELPER_HPP___

#include <armadillo>

namespace mr
{
/// \brief Determine if a value is near zero
/// \param z The value to be tested
/// \return true if near zero, othervise false
constexpr bool near_zero(const double z)
{
  return std::abs(z) < 1e-8;
}

static_assert(near_zero(0), "near_zero failed");

/// \brief Find the norm of the vector
/// \param V The input vector
/// \return The norm of the vector
double magnitute(const arma::vec & V);

/// \brief Normalize a vector
/// \param V The vector to be normalized
/// \return The nornamlized vector
const arma::vec Normalize(const arma::vec & V);

/// \brief Compute error
/// \param V Twist vector
/// \return True if error, False if success
bool compute_err(const arma::vec6 & V, const double emog, const double ev);
}

#endif /// MODERN_ROBOTICS__HELPER_HPP___
