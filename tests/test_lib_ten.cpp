#include <lib_ten.hpp>

#include "gtest/gtest.h"

TEST(ResidualNetwork, capacity)
{
  auto network = LibTEN::ResidualNetwork();
  auto p = network.source;
  auto q = network.sink;
  q->addParent(p);

  ASSERT_EQ(network.getCapacity(p, q), 1);
  ASSERT_EQ(network.getCapacity(q, p), 0);

  network.decrement(p, q);
  network.increment(q, p);
  ASSERT_EQ(network.getCapacity(p, q), 0);
  ASSERT_EQ(network.getCapacity(q, p), 1);

  network.setFlow(q, p);
  ASSERT_EQ(network.getCapacity(p, q), 1);
  ASSERT_EQ(network.getCapacity(q, p), 0);
}

TEST(ResidualNetwork, name)
{
  auto network = LibTEN::ResidualNetwork();
  auto p = network.source;
  auto q = network.sink;
  q->addParent(p);
  std::string edge_name = network.getEdgeName(p, q);

  ASSERT_EQ(network.getReverseEdgeName(edge_name), network.getEdgeName(q, p));
}
