#include "modern_robotics/helper.hpp"

namespace mr
{
double magnitute(const arma::vec & V)
{
  double sqr_sum = 0;
  for (auto & e : V) {
    sqr_sum += pow(e, 2.0);
  }

  return sqrt(sqr_sum);
}

const arma::vec Normalize(const arma::vec & V)
{
  const double mag = magnitute(V);
  return V / mag;
}

bool compute_err(const arma::vec6 & V, const double eomg, const double ev)
{
  const arma::colvec3 Vomg = V.subvec(0, 2);
  const arma::colvec3 Vv = V.subvec(3, 5);

  const double mag_omg = magnitute(Vomg);
  const double mag_v = magnitute(Vv);

  return mag_omg > eomg || mag_v > ev;
}
}
