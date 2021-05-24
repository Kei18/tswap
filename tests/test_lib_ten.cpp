#include <lib_ten.hpp>
#include <problem.hpp>

#include "gtest/gtest.h"

TEST(ResidualNetwork, capacity)
{
  Problem P = Problem("../tests/instances/03.txt");
  auto network = LibTEN::ResidualNetwork(false, &P);
  auto p = network.source;
  auto q = network.sink;
  q->addParent(p);

  ASSERT_EQ(network.getCapacity(p, q), 1);

  network.setFlow(p, q);
  ASSERT_EQ(network.getCapacity(p, q), 0);

  network.setFlow(q, p);
  ASSERT_EQ(network.getCapacity(p, q), 1);
}
