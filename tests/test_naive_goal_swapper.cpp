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

TEST(NaiveGoalSwapper, deadlock)
{
  Problem P = Problem("../tests/instances/10.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NaiveGoalSwapper>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(NaiveGoalSwapper, use_bfs)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NaiveGoalSwapper>(&P);
  char argv0[] = "dummy";
  char argv1[] = "-b";
  char* argv[] = {argv0, argv1};
  solver->setParams(2, argv);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
