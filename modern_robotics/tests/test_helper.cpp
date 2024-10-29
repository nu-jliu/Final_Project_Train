#include <catch2/catch_all.hpp>
#include "modern_robotics/helper.hpp"

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
