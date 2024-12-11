#include <catch2/catch_all.hpp>

#include "modern_robotics/inverse_kinematics.hpp"

constexpr double tolerance = 1e-6;

TEST_CASE("Testing inverse kinematics body", "[IKinBody]")
{
  const arma::mat Blist = arma::mat{
    {0, 0, -1, 2, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 1, 0, 0, 0.1}
  }.t();
  const arma::mat44 M{
    {-1, 0, 0, 0},
    {0, 1, 0, 6},
    {0, 0, -1, 2},
    {0, 0, 0, 1}
  };
  const arma::mat44 T{
    {0, 1, 0, -5},
    {1, 0, 0, 4},
    {0, 0, -1, 1.6858},
    {0, 0, 0, 1}
  };
  const std::vector<double> thetalist0{1.5, 2.5, 3};

  const auto result = mr::IKinBody(Blist, M, T, thetalist0);
  const arma::colvec thetalist = std::get<0>(result);
  const bool success = std::get<1>(result);

  // std::cout << thetalist << std::endl;
  CHECK(success);
  CHECK_THAT(thetalist.at(0), Catch::Matchers::WithinAbs(1.57073819, tolerance));
  CHECK_THAT(thetalist.at(1), Catch::Matchers::WithinAbs(2.999667, tolerance));
  CHECK_THAT(thetalist.at(2), Catch::Matchers::WithinAbs(3.14153913, tolerance));
}

TEST_CASE("Testing inverse kinematic space", "[IKinSpace]")
{
  const arma::mat Slist = arma::mat{
    {0, 0, 1, 4, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, -1, -6, 0, -0.1}
  }.t();
  const arma::mat44 M{
    {-1, 0, 0, 0},
    {0, 1, 0, 6},
    {0, 0, -1, 2},
    {0, 0, 0, 1}
  };
  const arma::mat44 T{
    {0, 1, 0, -5},
    {1, 0, 0, 4},
    {0, 0, -1, 1.6858},
    {0, 0, 0, 1}
  };
  const std::vector<double> thetalist0{1.5, 2.5, 3};
  const double emog = 0.01;
  const double ev = 0.001;

  const auto result = mr::IKinSpace(Slist, M, T, thetalist0, emog, ev);
  const arma::colvec thetalist = std::get<0>(result);
  const bool success = std::get<1>(result);

  // std::cout << thetalist << std::endl;
  CHECK(success);
  CHECK_THAT(thetalist.at(0), Catch::Matchers::WithinAbs(1.57073783, tolerance));
  CHECK_THAT(thetalist.at(1), Catch::Matchers::WithinAbs(2.99966384, tolerance));
  CHECK_THAT(thetalist.at(2), Catch::Matchers::WithinAbs(3.1415342, tolerance));
}
