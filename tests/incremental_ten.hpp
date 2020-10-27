#pragma once
#include "gtest/gtest.h"
#include <problem.hpp>
#include <incremental_time_expanded_network.hpp>

TEST(IncrementalTimeExpandedNetwork, update)
{
  Problem* P = new Problem("../tests/instances/03.txt");
  auto network = IncrementalTimeExpandedNetwork(P);

  network.update();
  ASSERT_EQ(network.nodesNum(), 16);
  ASSERT_EQ(network.edgesNum(), 23);
  ASSERT_FALSE(network.isValid());

  network.update();
  ASSERT_EQ(network.nodesNum(), 30);
  ASSERT_EQ(network.edgesNum(), 46);
  ASSERT_TRUE(network.isValid());
  ASSERT_EQ(network.getPlan().getSOC(), 3);

  delete P;
}
