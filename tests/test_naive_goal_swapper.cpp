#include <naive_goal_swapper.hpp>

#include "gtest/gtest.h"

TEST(NaiveGoalSwapper, solve)
{
  Problem P = Problem("../tests/instances/07.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NaiveGoalSwapper>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
