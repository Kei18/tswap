#include <goal_swapper.hpp>

#include "gtest/gtest.h"

TEST(GoalSwapper, solve)
{
  Problem P = Problem("../tests/instances/09.txt");
  std::unique_ptr<Solver> solver = std::make_unique<GoalSwapper>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(GoalSwapper, deadlock)
{
  Problem P = Problem("../tests/instances/10.txt");
  std::unique_ptr<Solver> solver = std::make_unique<GoalSwapper>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(GoalSwapper, use_bfs)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<GoalSwapper>(&P);
  char argv0[] = "dummy";
  char argv1[] = "-b";
  char* argv[] = {argv0, argv1};
  solver->setParams(2, argv);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
