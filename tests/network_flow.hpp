#pragma once
#include "gtest/gtest.h"
#include <network_flow.hpp>

TEST(NetworkFlow, network_flow)
{
  Problem* P = new Problem("../tests/instances/02.txt");
  Solver* solver = new NetworkFlow(P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(P));

  delete P;
  delete solver;
}
