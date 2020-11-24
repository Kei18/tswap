#include <network_flow.hpp>

#include "gtest/gtest.h"

TEST(NetworkFlow, TEN_INCREMENTAL)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NetworkFlow>(&P);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
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

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}

TEST(NetworkFlow, TEN_INCREMENTAL_USE_INSTRUCTED_TIMESTEP)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NetworkFlow>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-t";
  char argv2[] = "16";
  char* argv[] = {argv0, argv1, argv2};
  solver->setParams(3, argv);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}

TEST(NetworkFlow, TEN_INCREMENTAL_USE_BINARY_SEARCH)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NetworkFlow>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-b";
  char* argv[] = {argv0, argv1};
  solver->setParams(2, argv);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}

TEST(NetworkFlow, TEN_INCREMENTAL_USE_BINARY_SEARCH_WITHOUT_CACHE)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NetworkFlow>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-b";
  char argv2[] = "-n";
  char* argv[] = {argv0, argv1, argv2};
  solver->setParams(3, argv);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
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

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
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

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}

#ifdef _GUROBI_
TEST(NetworkFlow, TEN_ILP)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NetworkFlow>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-n";
  char argv2[] = "-g";
  char argv3[] = "-m";
  char* argv[] = {argv0, argv1, argv2, argv3};
  solver->setParams(4, argv);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}

TEST(NetworkFlow, TEN_INCREMENTAL_ILP)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<NetworkFlow>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-g";
  char argv2[] = "-m";
  char* argv[] = {argv0, argv1, argv2};
  solver->setParams(3, argv);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}
#endif
