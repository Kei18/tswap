#include <lib_ten.hpp>

#include "gtest/gtest.h"

TEST(ResidualNetwork, capacity)
{
  auto network = LibTEN::ResidualNetwork();
  auto p = network.source;
  auto q = network.sink;
  q->addParent(p);

  ASSERT_EQ(network.getCapacity(p, q), 1);

  network.setFlow(p, q);
  ASSERT_EQ(network.getCapacity(p, q), 0);

  network.setFlow(q, p);
  ASSERT_EQ(network.getCapacity(p, q), 1);
}
