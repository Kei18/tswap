#include <ir.hpp>

#include "gtest/gtest.h"

TEST(IR, solve)
{
  Problem P = Problem("../instances/sample.txt");
  std::unique_ptr<Solver> solver = std::make_unique<IR>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-n 10";
  char argv2[] = "-t 3000";
  char* argv[] = {argv0, argv1, argv2};
  solver->setParams(3, argv);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
