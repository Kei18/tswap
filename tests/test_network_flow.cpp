#include <network_flow.hpp>

#include "gtest/gtest.h"

TEST(NetworkFlow, TEN_INCREMENTAL)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NetworkFlow>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(NetworkFlow, TEN_INCREMENTAL_USE_MINIMUM_STEP)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NetworkFlow>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-m";
  char* argv[] = {argv0, argv1};
  solver->setParams(2, argv);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(NetworkFlow, TEN_INCREMENTAL_NO_FILTER)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NetworkFlow>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-f";
  char* argv[] = {argv0, argv1};
  solver->setParams(2, argv);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(NetworkFlow, TEN)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NetworkFlow>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-n";
  char argv2[] = "-f";
  char* argv[] = {argv0, argv1, argv2};
  solver->setParams(3, argv);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
