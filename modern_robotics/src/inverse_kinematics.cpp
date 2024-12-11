#include "modern_robotics/helper.hpp"
#include "modern_robotics/utils.hpp"
#include "modern_robotics/jacobian.hpp"
#include "modern_robotics/forward_kinematics.hpp"
#include "modern_robotics/inverse_kinematics.hpp"

namespace mr
{
constexpr int maxIter = 100;

const arma::vec6 compute_Vb(
  const arma::mat & Blist,
  const arma::mat44 & M,
  const arma::mat44 & T,
  const arma::colvec & thetalist
)
{
  const arma::mat44 Te = FKinBody(M, Blist, thetalist);
  const arma::mat44 Tb = TransInv(Te) * T;
  const arma::vec6 Vb = se3ToVec(MatrixLog6(Tb));

  return Vb;
}

const arma::vec6 compute_Vs(
  const arma::mat44 & Tsb,
  const arma::mat44 & T
)
{
  const arma::mat66 AdTsb = Adjoint(Tsb);
  const arma::mat44 Tbd = TransInv(Tsb) * T;
  const arma::vec6 Vs = AdTsb * se3ToVec(MatrixLog6(Tbd));

  return Vs;
}

const std::tuple<arma::colvec, bool> IKinBody(
  const arma::mat & Blist,
  const arma::mat44 & M,
  const arma::mat44 & T,
  const arma::colvec & thetalist0,
  const double emog,
  const double ev
)
{
  arma::colvec thetalist = thetalist0;
  int i = 0;

  arma::vec6 Vb = compute_Vb(Blist, M, T, thetalist);
  bool err = compute_err(Vb, emog, ev);

  while (err && i < maxIter) {
    const arma::vec dthetalist = arma::pinv(JacobianBody(Blist, thetalist)) * Vb;

    thetalist += dthetalist;
    ++i;

    Vb = compute_Vb(Blist, M, T, thetalist);
    err = compute_err(Vb, emog, ev);

    // std::cout << "Iter " << i << ": " << dthetalist << std::endl;
  }

  return {thetalist, !err};
}

const std::tuple<arma::colvec, bool> IKinSpace(
  const arma::mat & Slist,
  const arma::mat44 & M,
  const arma::mat44 & T,
  const arma::colvec & thetalist0,
  const double emog,
  const double ev
)
{
  arma::colvec thetalist(thetalist0);
  int i = 0;

  arma::mat44 Tsb = FKinSpace(M, Slist, thetalist);
  arma::vec6 Vs = compute_Vs(Tsb, T);

  bool err = compute_err(Vs, emog, ev);

  while (err && i < maxIter) {
    const arma::vec dthetalist = arma::pinv(JacobianSpace(Slist, thetalist)) * Vs;

    thetalist += dthetalist;
    ++i;

    Tsb = FKinSpace(M, Slist, thetalist);
    Vs = compute_Vs(Tsb, T);
    err = compute_err(Vs, emog, ev);
  }

  return {thetalist, !err};
}
}
