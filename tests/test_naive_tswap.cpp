#include <naive_tswap.hpp>

#include "gtest/gtest.h"

TEST(NaiveTSWAP, solve)
{
  Problem P = Problem("../tests/instances/07.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NaiveTSWAP>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(NaiveTSWAP, deadlock)
{
  Problem P = Problem("../tests/instances/10.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NaiveTSWAP>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
