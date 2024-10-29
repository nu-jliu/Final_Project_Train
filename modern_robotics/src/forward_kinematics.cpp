#include "modern_robotics/utils.hpp"
#include "modern_robotics/forward_kinematics.hpp"

namespace mr
{
const arma::mat44 FKinBody(
  const arma::mat44 & M, const arma::mat & Blist,
  const arma::colvec & thetalist
)
{
  arma::mat44 T(M);
  for (size_t i = 0; i < thetalist.size(); ++i) {
    const auto theta = thetalist.at(i);
    const arma::colvec Bvec = Blist.col(i) * theta;

    const arma::mat44 se3mat = VecTose3(Bvec);
    const arma::mat44 exp6mat = MatrixExp6(se3mat);

    T = T * exp6mat;
  }

  return T;
}

const arma::mat44 FKinSpace(
  const arma::mat44 & M, const arma::mat & Slist,
  const arma::colvec & thetalist
)
{
  arma::mat44 T(M);
  for (int i = static_cast<int>(thetalist.size()) - 1; i >= 0; --i) {
    const auto theta = thetalist.at(i);
    const arma::colvec Svec = Slist.col(i) * theta;

    const arma::mat44 se3mat = VecTose3(Svec);
    const arma::mat44 exp6mat = MatrixExp6(se3mat);

    T = exp6mat * T;
  }

  return T;
}
}
