#include "modern_robotics/jacobian.hpp"
#include "modern_robotics/utils.hpp"

namespace mr
{
const arma::mat JacobianBody(const arma::mat & Blist, const arma::colvec & thetalist)
{
  arma::mat Jb = Blist;
  arma::mat44 T = arma::eye(4, 4);

  for (int i = static_cast<int>(thetalist.size()) - 2; i >= 0; --i) {
    T = T * MatrixExp6(VecTose3(Blist.col(i + 1) * -thetalist.at(i + 1)));
    Jb.col(i) = Adjoint(T) * Blist.col(i);
  }

  return Jb;
}

const arma::mat JacobianSpace(const arma::mat & Slist, const arma::colvec & thetalist)
{
  arma::mat Js = Slist;
  arma::mat44 T = arma::eye(4, 4);

  for (size_t i = 1; i < thetalist.size(); ++i) {
    T = T * MatrixExp6(VecTose3(Slist.col(i - 1) * thetalist.at(i - 1)));
    Js.col(i) = Adjoint(T) * Slist.col(i);
  }

  return Js;
}
}
