#include <catch2/catch_all.hpp>

#include "modern_robotics/jacobian.hpp"

constexpr double tolerance = 1e-8;

TEST_CASE("Test jacobian body", "[JacobianBody]")
{
  const arma::mat Blist = arma::mat{
    {0, 0, 1, 0, 0.2, 0.2},
    {1, 0, 0, 2, 0, 3},
    {0, 1, 0, 0, 2, 1},
    {1, 0, 0, 0.2, 0.3, 0.4}
  }.t();
  const std::vector<double> thetalist{0.2, 1.1, 0.1, 1.2};

  const arma::mat Jb = mr::JacobianBody(Blist, thetalist);
  // std::cout << Jb << std::endl;

  CHECK_THAT(Jb.at(0, 0), Catch::Matchers::WithinAbs(-0.04528405, tolerance));
  CHECK_THAT(Jb.at(0, 1), Catch::Matchers::WithinAbs(0.99500417, tolerance));
  CHECK_THAT(Jb.at(0, 2), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Jb.at(0, 3), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(Jb.at(1, 0), Catch::Matchers::WithinAbs(0.74359313, tolerance));
  CHECK_THAT(Jb.at(1, 1), Catch::Matchers::WithinAbs(0.09304865, tolerance));
  CHECK_THAT(Jb.at(1, 2), Catch::Matchers::WithinAbs(0.36235775, tolerance));
  CHECK_THAT(Jb.at(1, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Jb.at(2, 0), Catch::Matchers::WithinAbs(-0.66709716, tolerance));
  CHECK_THAT(Jb.at(2, 1), Catch::Matchers::WithinAbs(0.03617541, tolerance));
  CHECK_THAT(Jb.at(2, 2), Catch::Matchers::WithinAbs(-0.93203909, tolerance));
  CHECK_THAT(Jb.at(2, 3), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Jb.at(3, 0), Catch::Matchers::WithinAbs(2.32586047, tolerance));
  CHECK_THAT(Jb.at(3, 1), Catch::Matchers::WithinAbs(1.66809, tolerance));
  CHECK_THAT(Jb.at(3, 2), Catch::Matchers::WithinAbs(0.56410831, tolerance));
  CHECK_THAT(Jb.at(3, 3), Catch::Matchers::WithinAbs(0.2, tolerance));
  CHECK_THAT(Jb.at(4, 0), Catch::Matchers::WithinAbs(-1.44321167, tolerance));
  CHECK_THAT(Jb.at(4, 1), Catch::Matchers::WithinAbs(2.94561275, tolerance));
  CHECK_THAT(Jb.at(4, 2), Catch::Matchers::WithinAbs(1.43306521, tolerance));
  CHECK_THAT(Jb.at(4, 3), Catch::Matchers::WithinAbs(0.3, tolerance));
  CHECK_THAT(Jb.at(5, 0), Catch::Matchers::WithinAbs(-2.06639565, tolerance));
  CHECK_THAT(Jb.at(5, 1), Catch::Matchers::WithinAbs(1.82881722, tolerance));
  CHECK_THAT(Jb.at(5, 2), Catch::Matchers::WithinAbs(-1.58868628, tolerance));
  CHECK_THAT(Jb.at(5, 3), Catch::Matchers::WithinAbs(0.4, tolerance));
}

TEST_CASE("Testing Jacobian", "[JacobianSpace]")
{
  const arma::mat Slist = arma::mat{
    {0, 0, 1, 0, 0.2, 0.2},
    {1, 0, 0, 2, 0, 3},
    {0, 1, 0, 0, 2, 1},
    {1, 0, 0, 0.2, 0.3, 0.4}
  }.t();
  const std::vector<double> thetalist{0.2, 1.1, 0.1, 1.2};

  const arma::mat Js = mr::JacobianSpace(Slist, thetalist);

  // std::cout << Js << std::endl;
  CHECK_THAT(Js.at(0, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Js.at(0, 1), Catch::Matchers::WithinAbs(0.98006658, tolerance));
  CHECK_THAT(Js.at(0, 2), Catch::Matchers::WithinAbs(-0.09011564, tolerance));
  CHECK_THAT(Js.at(0, 3), Catch::Matchers::WithinAbs(0.95749426, tolerance));
  CHECK_THAT(Js.at(1, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Js.at(1, 1), Catch::Matchers::WithinAbs(0.19866933, tolerance));
  CHECK_THAT(Js.at(1, 2), Catch::Matchers::WithinAbs(0.4445544, tolerance));
  CHECK_THAT(Js.at(1, 3), Catch::Matchers::WithinAbs(0.28487557, tolerance));
  CHECK_THAT(Js.at(2, 0), Catch::Matchers::WithinAbs(1.0, tolerance));
  CHECK_THAT(Js.at(2, 1), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Js.at(2, 2), Catch::Matchers::WithinAbs(0.89120736, tolerance));
  CHECK_THAT(Js.at(2, 3), Catch::Matchers::WithinAbs(-0.04528405, tolerance));
  CHECK_THAT(Js.at(3, 0), Catch::Matchers::WithinAbs(0.0, tolerance));
  CHECK_THAT(Js.at(3, 1), Catch::Matchers::WithinAbs(1.95218638, tolerance));
  CHECK_THAT(Js.at(3, 2), Catch::Matchers::WithinAbs(-2.21635216, tolerance));
  CHECK_THAT(Js.at(3, 3), Catch::Matchers::WithinAbs(-0.51161537, tolerance));
  CHECK_THAT(Js.at(4, 0), Catch::Matchers::WithinAbs(0.2, tolerance));
  CHECK_THAT(Js.at(4, 1), Catch::Matchers::WithinAbs(0.43654132, tolerance));
  CHECK_THAT(Js.at(4, 2), Catch::Matchers::WithinAbs(-2.43712573, tolerance));
  CHECK_THAT(Js.at(4, 3), Catch::Matchers::WithinAbs(2.77535713, tolerance));
  CHECK_THAT(Js.at(5, 0), Catch::Matchers::WithinAbs(0.2, tolerance));
  CHECK_THAT(Js.at(5, 1), Catch::Matchers::WithinAbs(2.96026613, tolerance));
  CHECK_THAT(Js.at(5, 2), Catch::Matchers::WithinAbs(3.23573065, tolerance));
  CHECK_THAT(Js.at(5, 3), Catch::Matchers::WithinAbs(2.22512443, tolerance));
}
