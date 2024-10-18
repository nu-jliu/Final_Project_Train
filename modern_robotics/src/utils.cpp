#include "modern_robotics/utils.hpp"
#include <cmath>

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

const std::tuple<const arma::mat33, const arma::colvec3> TransToRp(const arma::mat44 & T)
{
  const arma::mat33 Rmat = T.submat(0, 0, 2, 2);
  const arma::colvec3 pvec = T.col(3).subvec(0, 2).as_col();

  return {Rmat, pvec};
}

const arma::mat33 VecToso3(const arma::vec3 & omg)
{
  const arma::mat33 so3mat{
    {0, -omg.at(2), omg.at(1)},
    {omg.at(2), 0, -omg.at(0)},
    {-omg.at(1), omg.at(0), 0}
  };

  return so3mat;
}

const arma::vec3 so3ToVec(const arma::mat33 & so3mat)
{
  const arma::vec3 omg{so3mat.at(2, 1), so3mat.at(0, 2), so3mat(1, 0)};
  return omg;
}

const std::tuple<const arma::vec3, double> AxisAng3(const arma::vec3 & expc3)
{
  const arma::vec3 V_norm = Normalize(expc3);
  const double norm_val = norm(expc3);

  return {V_norm, norm_val};
}

const arma::mat44 VecTose3(const arma::vec6 & V)
{
  const arma::vec3 omg = V.subvec(0, 2);
  const arma::mat33 so3mat = VecToso3(omg);
  const arma::colvec3 pvec = V.subvec(3, 5).as_col();

  const arma::rowvec4 lower(arma::fill::zeros);
  const arma::mat upper = arma::join_horiz(so3mat, pvec);

  const arma::mat44 se3mat = arma::join_vert(upper, lower);
  return se3mat;
}

const arma::mat66 Adjoint(const arma::mat44 & T)
{
  const auto Rp = TransToRp(T);
  const arma::mat33 Rmat = std::get<0>(Rp);
  const arma::colvec3 pvec = std::get<1>(Rp);

  const arma::mat33 so3mat = VecToso3(pvec);
  const arma::mat33 pRmat = so3mat * Rmat;

  const arma::mat upper = arma::join_horiz(Rmat, arma::mat33{arma::fill::zeros});
  const arma::mat lower = arma::join_horiz(pRmat, Rmat);

  const arma::mat66 AdTmat = arma::join_vert(upper, lower);
  return AdTmat;
}

const arma::mat33 MatrixExp3(const arma::mat33 & so3mat)
{
  const arma::vec3 omgtheta = so3ToVec(so3mat);

  if (near_zero(magnitute(omgtheta))) {
    const arma::mat33 exp3mat{arma::fill::eye};
    return exp3mat;
  } else {
    const auto theta = std::get<1>(AxisAng3(omgtheta));
    const arma::mat33 omgmat = so3mat / theta;

    const arma::mat33 identity{arma::fill::eye};
    const arma::mat33 exp3mat = identity + sin(theta) * omgmat + (1 - cos(theta)) *
      (omgmat * omgmat);

    return exp3mat;
  }
}

const arma::mat44 MatrixExp6(const arma::mat44 se3mat)
{
  const arma::vec3 omgtheta = so3ToVec(se3mat.submat(0, 0, 2, 2));

  if (near_zero(magnitute(omgtheta))) {
    const arma::mat33 identity{arma::fill::eye};
    const arma::colvec3 pvec = se3mat.col(3).subvec(0, 2).as_col();

    const arma::mat44 exp6mat = arma::join_vert(
      arma::join_horiz(
        identity,
        pvec
      ),
      arma::rowvec4{0, 0, 0, 1}
    );

    return exp6mat;
  } else {
    const auto theta = std::get<1>(AxisAng3(omgtheta));
    const arma::mat33 omgmat = se3mat.submat(0, 0, 2, 2) / theta;

    const arma::mat33 Imat{arma::fill::eye};
    const arma::mat33 Rmat = MatrixExp3(se3mat.submat(0, 0, 2, 2));

    const arma::colvec3 pvec =
      (Imat * theta +
      (1 - cos(theta)) * omgmat +
      (theta - sin(theta)) * (omgmat * omgmat)) *
      (se3mat.submat(0, 3, 2, 3).as_col() / theta);

    const arma::mat upper = arma::join_horiz(Rmat, pvec);
    const arma::rowvec4 lower{0, 0, 0, 1};

    const arma::mat44 exp6mat = arma::join_vert(upper, lower);
    return exp6mat;
  }
}
}
