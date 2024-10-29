#include <cmath>

#include "modern_robotics/helper.hpp"
#include "modern_robotics/utils.hpp"

namespace mr
{
const std::tuple<const arma::mat33, const arma::colvec3> TransToRp(const arma::mat44 & T)
{
  const arma::mat33 Rmat = T.submat(0, 0, 2, 2);
  const arma::colvec3 pvec = T.col(3).subvec(0, 2).as_col();

  return {Rmat, pvec};
}

const arma::mat44 TransInv(const arma::mat44 & T)
{
  const auto Rp = mr::TransToRp(T);
  const arma::mat33 R = std::get<0>(Rp);
  const arma::colvec3 p = std::get<1>(Rp);

  const arma::mat33 Rt = R.t();

  const arma::colvec3 upper_right = -Rt * p;
  const arma::mat upper = arma::join_horiz(Rt, upper_right);
  const arma::rowvec4 lower{0, 0, 0, 1};

  return arma::join_vert(upper, lower);
}

const arma::mat33 VecToso3(const arma::vec3 & omg)
{
  return {
    {0, -omg.at(2), omg.at(1)},
    {omg.at(2), 0, -omg.at(0)},
    {-omg.at(1), omg.at(0), 0}
  };
}

const arma::vec3 so3ToVec(const arma::mat33 & so3mat)
{
  return {so3mat.at(2, 1), so3mat.at(0, 2), so3mat(1, 0)};
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

const arma::vec6 se3ToVec(const arma::mat44 & se3mat)
{
  return {
    se3mat.at(2, 1),
    se3mat.at(0, 2),
    se3mat.at(1, 0),
    se3mat.at(0, 3),
    se3mat.at(1, 3),
    se3mat.at(2, 3)
  };
}

const arma::mat66 Adjoint(const arma::mat44 & T)
{
  const auto Rp = TransToRp(T);
  const arma::mat33 Rmat = std::get<0>(Rp);
  const arma::colvec3 pvec = std::get<1>(Rp);

  const arma::mat33 so3mat = VecToso3(pvec);
  const arma::mat33 pRmat = so3mat * Rmat;

  const arma::mat upper = arma::join_horiz(Rmat, arma::zeros(3, 3));
  const arma::mat lower = arma::join_horiz(pRmat, Rmat);

  const arma::mat66 AdTmat = arma::join_vert(upper, lower);
  return AdTmat;
}

const arma::mat33 MatrixExp3(const arma::mat33 & so3mat)
{
  const arma::vec3 omgtheta = so3ToVec(so3mat);

  if (near_zero(magnitute(omgtheta))) {
    return {arma::fill::eye};
  } else {
    const auto theta = std::get<1>(AxisAng3(omgtheta));
    const arma::mat33 omgmat = so3mat / theta;

    const arma::mat33 identity{arma::fill::eye};
    const arma::mat33 exp3mat = identity + sin(theta) * omgmat + (1 - cos(theta)) *
      (omgmat * omgmat);

    return exp3mat;
  }
}

const arma::mat33 MatrixLog3(const arma::mat33 & R)
{
  const double acosinput = (arma::trace(R) - 1) / 2.0;

  if (acosinput >= 1) {
    return {arma::fill::zeros};
  } else if (acosinput <= -1) {
    arma::vec3 omg;

    if (!near_zero(1 + R.at(2, 2))) {
      omg = (1.0 / sqrt(2 * (1 + R.at(2, 2)))) *
        arma::vec3{R.at(0, 2), R.at(1, 2), R.at(2, 2)};
    } else if (!near_zero(1 + R.at(1, 1))) {
      omg = (1.0 / sqrt(2 * (1 + R.at(1, 1)))) *
        arma::vec3{R.at(0, 1), R.at(1, 1), R.at(2, 1)};
    } else {
      omg = (1.0 / sqrt(2 * (1 + R.at(0, 0)))) *
        arma::vec3{R.at(0, 0), R.at(1, 0), R.at(2, 0)};
    }

    return VecToso3(M_PI * omg);
  } else {
    const double theta = acos(acosinput);
    return theta / 2.0 / sin(theta) * (R - R.t());
  }
}

const arma::mat44 MatrixExp6(const arma::mat44 & se3mat)
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

const arma::mat44 MatrixLog6(const arma::mat44 & T)
{
  const auto Rp = mr::TransToRp(T);
  const arma::mat33 R = std::get<0>(Rp);
  const arma::colvec3 p = std::get<1>(Rp);
  const arma::mat33 omgmat = MatrixLog3(R);

  if (arma::approx_equal(omgmat, arma::zeros(3, 3), "absdiff", 1e-9)) {
    const arma::mat upper = arma::join_horiz(
      arma::zeros(3, 3),
      arma::colvec3{T.at(0, 3), T.at(1, 3), T.at(2, 3)}
    );
    const arma::rowvec4 lower{arma::fill::zeros};

    return arma::join_vert(upper, lower);
  } else {
    const double theta = acos((arma::trace(R) - 1) / 2.0);
    const arma::colvec3 upper_right =
      (arma::eye(3, 3) - omgmat / 2.0 +
      (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2.0) *
      (omgmat * omgmat) / theta) *
      arma::colvec3{T.at(0, 3), T.at(1, 3), T.at(2, 3)};
    const arma::mat upper = arma::join_horiz(omgmat, upper_right);
    const arma::rowvec4 lower{arma::fill::zeros};

    return arma::join_vert(upper, lower);
  }
}
}
