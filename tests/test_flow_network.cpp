#include <flow_network.hpp>

#include "gtest/gtest.h"

TEST(FlowNetwork, TEN_INCREMENTAL)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<FlowNetwork>(&P);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}

TEST(FlowNetwork, TEN_INCREMENTAL_USE_LOWER_BOUND)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<FlowNetwork>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-l";
  char* argv[] = {argv0, argv1};
  solver->setParams(2, argv);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}

TEST(FlowNetwork, TEN_INCREMENTAL_USE_INSTRUCTED_TIMESTEP)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<FlowNetwork>(&P);

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

TEST(FlowNetwork, TEN_INCREMENTAL_USE_BINARY_SEARCH)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<FlowNetwork>(&P);

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

TEST(FlowNetwork, TEN_INCREMENTAL_USE_BINARY_SEARCH_WITHOUT_CACHE)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<FlowNetwork>(&P);

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

TEST(FlowNetwork, TEN_INCREMENTAL_NO_PRUNING)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<FlowNetwork>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-p";
  char* argv[] = {argv0, argv1};
  solver->setParams(2, argv);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}

TEST(FlowNetwork, TEN)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<FlowNetwork>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-n";
  char argv2[] = "-p";
  char* argv[] = {argv0, argv1, argv2};
  solver->setParams(3, argv);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}

#ifdef _GUROBI_
TEST(FlowNetwork, TEN_ILP)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<FlowNetwork>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-n";
  char argv2[] = "-g";
  char argv3[] = "-l";
  char* argv[] = {argv0, argv1, argv2, argv3};
  solver->setParams(4, argv);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}

TEST(FlowNetwork, TEN_INCREMENTAL_ILP)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<FlowNetwork>(&P);

  char argv0[] = "dummy";
  char argv1[] = "-g";
  char argv2[] = "-l";
  char* argv[] = {argv0, argv1, argv2};
  solver->setParams(3, argv);
  solver->solve();

  auto plan = solver->getSolution();
  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getMakespan() == 16);
}
#endif
