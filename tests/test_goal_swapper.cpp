#include <goal_swapper.hpp>

#include "gtest/gtest.h"

TEST(GoalSwapper, solve)
{
  Problem P = Problem("../instances/arena_500agents_1.txt");
  std::unique_ptr<Solver> solver = std::make_unique<GoalSwapper>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
