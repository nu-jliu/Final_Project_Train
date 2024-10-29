#include <catch2/catch_all.hpp>
#include "modern_robotics/utils.hpp"

constexpr double tolerance = 1e-6;

TEST_CASE("Test Transform matrix to Rotation matrix and p vector", "[TransToRp]")
{
  const arma::mat44 T = {
    {1, 0, 0, 0},
    {0, 0, -1, 0},
    {0, 1, 0, 3},
    {0, 0, 0, 1}
  };

  const auto Rp = mr::TransToRp(T);
  const arma::mat33 R = std::get<0>(Rp);
  const arma::colvec3 p = std::get<1>(Rp);

  CHECK_THAT(R.at(0, 0), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(R.at(0, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(R.at(0, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(R.at(1, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(R.at(1, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(R.at(1, 2), Catch::Matchers::WithinAbs(-1.0, tolerance));
  CHECK_THAT(R.at(2, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(R.at(2, 1), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(R.at(2, 2), Catch::Matchers::WithinAbs(0.0, tolerance));

  CHECK_THAT(p.at(0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(p.at(1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(p.at(2), Catch::Matchers::WithinAbs(3.0, tolerance));
}

TEST_CASE("Testing homogenous transform matrix inverse", "[TransInv]")
{
  const arma::mat44 T{
    {1, 0, 0, 0},
    {0, 0, -1, 0},
    {0, 1, 0, 3},
    {0, 0, 0, 1}
  };

  const arma::mat44 Tinv = mr::TransInv(T);

  // std::cout << Tinv << std::endl;
  CHECK_THAT(Tinv.at(0, 0), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(Tinv.at(0, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(0, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(0, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(1, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(1, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(1, 2), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(Tinv.at(1, 3), Catch::Matchers::WithinAbs(-3.0, tolerance));
  CHECK_THAT(Tinv.at(2, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(2, 1), Catch::Matchers::WithinAbs(-1.0, tolerance));
  CHECK_THAT(Tinv.at(2, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(2, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(3, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(3, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(3, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Tinv.at(3, 3), Catch::Matchers::WithinAbs(1.0, tolerance));
}

TEST_CASE("Test vector to so3 matrix", "[VecToso3]")
{
  const arma::vec3 omg1{1, 2, 3};
  const arma::vec3 omg2{4, 5, 6};

  const arma::mat33 so3mat1 = mr::VecToso3(omg1);
  const arma::mat33 so3mat2 = mr::VecToso3(omg2);
  // std::cout << so3mat1 << std::endl;

  CHECK_THAT(so3mat1.at(0, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(so3mat1.at(0, 1), Catch::Matchers::WithinAbs(-3.0, tolerance));
  CHECK_THAT(so3mat1.at(0, 2), Catch::Matchers::WithinAbs(2.0, tolerance));
  CHECK_THAT(so3mat1.at(1, 0), Catch::Matchers::WithinAbs(3.0, tolerance));
  CHECK_THAT(so3mat1.at(1, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(so3mat1.at(1, 2), Catch::Matchers::WithinAbs(-1.0, tolerance));
  CHECK_THAT(so3mat1.at(2, 0), Catch::Matchers::WithinAbs(-2.0, tolerance));
  CHECK_THAT(so3mat1.at(2, 1), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(so3mat1.at(2, 2), Catch::Matchers::WithinAbs(0.0, tolerance));

  CHECK_THAT(so3mat2.at(0, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(so3mat2.at(0, 1), Catch::Matchers::WithinAbs(-6.0, tolerance));
  CHECK_THAT(so3mat2.at(0, 2), Catch::Matchers::WithinAbs(5.0, tolerance));
  CHECK_THAT(so3mat2.at(1, 0), Catch::Matchers::WithinAbs(6.0, tolerance));
  CHECK_THAT(so3mat2.at(1, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(so3mat2.at(1, 2), Catch::Matchers::WithinAbs(-4.0, tolerance));
  CHECK_THAT(so3mat2.at(2, 0), Catch::Matchers::WithinAbs(-5.0, tolerance));
  CHECK_THAT(so3mat2.at(2, 1), Catch::Matchers::WithinAbs(4.0, tolerance));
  CHECK_THAT(so3mat2.at(2, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
}

TEST_CASE("Test so3 matrix to vector", "[so3ToVec]")
{
  const arma::mat33 so3mat{
    {0, -3, 2},
    {3, 0, -1},
    {-2, 1, 0}
  };

  const arma::vec3 omg = mr::so3ToVec(so3mat);
  // std::cout << omg << std::endl;

  CHECK_THAT(omg.at(0), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(omg.at(1), Catch::Matchers::WithinAbs(2.0, tolerance));
  CHECK_THAT(omg.at(2), Catch::Matchers::WithinAbs(3.0, tolerance));
}

TEST_CASE("Test axis angle", "[AxisAng3]")
{
  const arma::vec3 expc3{1, 2, 3};
  const auto omgtheta = mr::AxisAng3(expc3);

  const arma::vec3 omghat = std::get<0>(omgtheta);
  const auto theta = std::get<1>(omgtheta);

  CHECK_THAT(omghat.at(0), Catch::Matchers::WithinAbs(0.26726124, tolerance));
  CHECK_THAT(omghat.at(1), Catch::Matchers::WithinAbs(0.53452248, tolerance));
  CHECK_THAT(omghat.at(2), Catch::Matchers::WithinAbs(0.80178373, tolerance));
  CHECK_THAT(theta, Catch::Matchers::WithinAbs(3.7416573867739413, tolerance));
}

TEST_CASE("Test vector to se3 matrix", "[VecTose3]")
{
  const arma::vec6 V1{1, 2, 3, 4, 5, 6};

  const arma::mat44 se3mat1 = mr::VecTose3(V1);
  // std::cout << se3mat1 << std::endl;

  CHECK_THAT(se3mat1.at(0, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat1.at(0, 1), Catch::Matchers::WithinAbs(-3.0, tolerance));
  CHECK_THAT(se3mat1.at(0, 2), Catch::Matchers::WithinAbs(2.0, tolerance));
  CHECK_THAT(se3mat1.at(0, 3), Catch::Matchers::WithinAbs(4.0, tolerance));
  CHECK_THAT(se3mat1.at(1, 0), Catch::Matchers::WithinAbs(3.0, tolerance));
  CHECK_THAT(se3mat1.at(1, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat1.at(1, 2), Catch::Matchers::WithinAbs(-1.0, tolerance));
  CHECK_THAT(se3mat1.at(1, 3), Catch::Matchers::WithinAbs(5.0, tolerance));
  CHECK_THAT(se3mat1.at(2, 0), Catch::Matchers::WithinAbs(-2.0, tolerance));
  CHECK_THAT(se3mat1.at(2, 1), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(se3mat1.at(2, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat1.at(2, 3), Catch::Matchers::WithinAbs(6.0, tolerance));
  CHECK_THAT(se3mat1.at(3, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat1.at(3, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat1.at(3, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat1.at(3, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
}

TEST_CASE("Test se3 martrix to vector", "[se3ToVec]")
{
  const arma::mat44 se3mat{
    {0, -3, 2, 4},
    {3, 0, -1, 5},
    {-2, 1, 0, 6},
    {0, 0, 0, 0}
  };

  const arma::vec6 V = mr::se3ToVec(se3mat);

  CHECK_THAT(V.at(0), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(V.at(1), Catch::Matchers::WithinAbs(2.0, tolerance));
  CHECK_THAT(V.at(2), Catch::Matchers::WithinAbs(3.0, tolerance));
  CHECK_THAT(V.at(3), Catch::Matchers::WithinAbs(4.0, tolerance));
  CHECK_THAT(V.at(4), Catch::Matchers::WithinAbs(5.0, tolerance));
  CHECK_THAT(V.at(5), Catch::Matchers::WithinAbs(6.0, tolerance));
}

TEST_CASE("Testing Adjoint", "[Adjoint]")
{
  const arma::mat44 T{
    {1, 0, 0, 0},
    {0, 0, -1, 0},
    {0, 1, 0, 3},
    {0, 0, 0, 1}
  };

  const arma::mat66 AdT = mr::Adjoint(T);

  CHECK_THAT(AdT.at(0, 0), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(AdT.at(0, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(0, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(0, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(0, 4), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(0, 5), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(1, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(1, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(1, 2), Catch::Matchers::WithinAbs(-1.0, tolerance));
  CHECK_THAT(AdT.at(1, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(1, 4), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(1, 5), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(2, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(2, 1), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(AdT.at(2, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(2, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(2, 4), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(2, 5), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(3, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(3, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(3, 2), Catch::Matchers::WithinAbs(3.0, tolerance));
  CHECK_THAT(AdT.at(3, 3), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(AdT.at(3, 4), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(3, 5), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(4, 0), Catch::Matchers::WithinAbs(3.0, tolerance));
  CHECK_THAT(AdT.at(4, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(4, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(4, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(4, 4), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(4, 5), Catch::Matchers::WithinAbs(-1.0, tolerance));
  CHECK_THAT(AdT.at(5, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(5, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(5, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(5, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(AdT.at(5, 4), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(AdT.at(5, 5), Catch::Matchers::WithinAbs(0.0, tolerance));
}

TEST_CASE("Test so3 matrix exponential", "[MatrixExp3]")
{
  const arma::mat33 so3mat{
    {0, -3, 2},
    {3, 0, -1},
    {-2, 1, 0}
  };

  const arma::mat33 exp3mat = mr::MatrixExp3(so3mat);

  CHECK_THAT(exp3mat.at(0, 0), Catch::Matchers::WithinAbs(-0.69492056, tolerance));
  CHECK_THAT(exp3mat.at(0, 1), Catch::Matchers::WithinAbs(0.71352099, tolerance));
  CHECK_THAT(exp3mat.at(0, 2), Catch::Matchers::WithinAbs(0.08929286, tolerance));
  CHECK_THAT(exp3mat.at(1, 0), Catch::Matchers::WithinAbs(-0.19200697, tolerance));
  CHECK_THAT(exp3mat.at(1, 1), Catch::Matchers::WithinAbs(-0.30378504, tolerance));
  CHECK_THAT(exp3mat.at(1, 2), Catch::Matchers::WithinAbs(0.93319235, tolerance));
  CHECK_THAT(exp3mat.at(2, 0), Catch::Matchers::WithinAbs(0.69297817, tolerance));
  CHECK_THAT(exp3mat.at(2, 1), Catch::Matchers::WithinAbs(0.6313497, tolerance));
  CHECK_THAT(exp3mat.at(2, 2), Catch::Matchers::WithinAbs(0.34810748, tolerance));
}

TEST_CASE("Test so3 matrix log", "[MatrixLog3]")
{
  const arma::mat33 R{
    {0, 0, 1},
    {1, 0, 0},
    {0, 1, 0}
  };

  const arma::mat33 so3mat = mr::MatrixLog3(R);

  CHECK_THAT(so3mat.at(0, 0), Catch::Matchers::WithinAbs(0, tolerance));
  CHECK_THAT(so3mat.at(0, 1), Catch::Matchers::WithinAbs(-1.20919958, tolerance));
  CHECK_THAT(so3mat.at(0, 2), Catch::Matchers::WithinAbs(1.20919958, tolerance));
  CHECK_THAT(so3mat.at(1, 0), Catch::Matchers::WithinAbs(1.20919958, tolerance));
  CHECK_THAT(so3mat.at(1, 1), Catch::Matchers::WithinAbs(0, tolerance));
  CHECK_THAT(so3mat.at(1, 2), Catch::Matchers::WithinAbs(-1.20919958, tolerance));
  CHECK_THAT(so3mat.at(2, 0), Catch::Matchers::WithinAbs(-1.20919958, tolerance));
  CHECK_THAT(so3mat.at(2, 1), Catch::Matchers::WithinAbs(1.20919958, tolerance));
  CHECK_THAT(so3mat.at(2, 2), Catch::Matchers::WithinAbs(0, tolerance));
}

TEST_CASE("Test se3 matrix exponential", "[MatrixExp6]")
{
  const arma::mat44 se3mat{
    {0, 0, 0, 0},
    {0, 0, -1.57079632, 2.35619449},
    {0, 1.57079632, 0, 2.35619449},
    {0, 0, 0, 0}
  };

  const arma::mat44 T = mr::MatrixExp6(se3mat);

  // std::cout << T << std::endl;
  CHECK_THAT(T.at(0, 0), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(T.at(0, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(0, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(0, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(1, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(1, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(1, 2), Catch::Matchers::WithinAbs(-1.0, tolerance));
  CHECK_THAT(T.at(1, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(2, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(2, 1), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(T.at(2, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(2, 3), Catch::Matchers::WithinAbs(3.0, tolerance));
  CHECK_THAT(T.at(3, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(3, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(3, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(T.at(3, 3), Catch::Matchers::WithinAbs(1.0, tolerance));
}

TEST_CASE("Test matrix logarithm of transformation matrix", "[MatrixLog6]")
{
  const arma::mat44 T {
    {1, 0, 0, 0},
    {0, 0, -1, 0},
    {0, 1, 0, 3},
    {0, 0, 0, 1}
  };

  const arma::mat44 se3mat = mr::MatrixLog6(T);
  // std::cout << se3mat << std::endl;

  CHECK_THAT(se3mat.at(0, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(0, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(0, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(0, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(1, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(1, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(1, 2), Catch::Matchers::WithinAbs(-1.57079633, tolerance));
  CHECK_THAT(se3mat.at(1, 3), Catch::Matchers::WithinAbs(2.35619449, tolerance));
  CHECK_THAT(se3mat.at(2, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(2, 1), Catch::Matchers::WithinAbs(1.57079633, tolerance));
  CHECK_THAT(se3mat.at(2, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(2, 3), Catch::Matchers::WithinAbs(2.35619449, tolerance));
  CHECK_THAT(se3mat.at(3, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(3, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(3, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(se3mat.at(3, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
}
