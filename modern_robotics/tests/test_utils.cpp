#include <catch2/catch_all.hpp>
#include "modern_robotics/utils.hpp"

constexpr double tolerance = 1e-6;

TEST_CASE("Test magnitute of a vector", "[magnitute]")
{
  const arma::vec V1{1, 2, 3};
  const arma::vec V2{2, 3, 4, 5, 6};

  const auto mag1 = mr::magnitute(V1);
  const auto mag2 = mr::magnitute(V2);

  CHECK_THAT(mag1, Catch::Matchers::WithinAbs(3.74165738, tolerance));
  CHECK_THAT(mag2, Catch::Matchers::WithinAbs(9.48683298, tolerance));
}

TEST_CASE("Test normalize vector", "[Normalize]")
{
  arma::vec V{1, 2, 3};

  arma::vec V_norm = mr::Normalize(V);

  // std::cout << vec1_norm << std::endl;
  // std::cout << vec2_norm << std::endl;
  CHECK(V_norm.size() == 3);
  CHECK_THAT(V_norm.at(0), Catch::Matchers::WithinAbs(0.26726124, tolerance));
  CHECK_THAT(V_norm.at(1), Catch::Matchers::WithinAbs(0.53452248, tolerance));
  CHECK_THAT(V_norm.at(2), Catch::Matchers::WithinAbs(0.80178373, tolerance));
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

TEST_CASE("Test se3 matrix exponential", "[MatrixExp6]")
{
  const arma::mat44 se3mat{
    {0, 0, 0, 0},
    {0, 0, -1.57079632, 2.35619449},
    {0, 1.57079632, 0, 2.35619449},
    {0, 0, 0, 0}
  };

  const arma::mat44 exp6mat = mr::MatrixExp6(se3mat);

  // std::cout << exp6mat << std::endl;
  CHECK_THAT(exp6mat.at(0, 0), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(exp6mat.at(0, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(0, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(0, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(1, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(1, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(1, 2), Catch::Matchers::WithinAbs(-1.0, tolerance));
  CHECK_THAT(exp6mat.at(1, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(2, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(2, 1), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(exp6mat.at(2, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(2, 3), Catch::Matchers::WithinAbs(3.0, tolerance));
  CHECK_THAT(exp6mat.at(3, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(3, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(3, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(exp6mat.at(3, 3), Catch::Matchers::WithinAbs(1.0, tolerance));
}
