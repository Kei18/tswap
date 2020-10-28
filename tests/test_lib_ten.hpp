#pragma once
#include <lib_ten.hpp>

#include "gtest/gtest.h"

TEST(ResidualNetwork, residual_capacity)
{
  auto p = LibTEN::TEN_Node::createNewNode(LibTEN::TEN_Node::NodeType::SOURCE);
  auto q = LibTEN::TEN_Node::createNewNode(LibTEN::TEN_Node::NodeType::SINK);
  q->addParent(p);

  auto network = LibTEN::ResidualNetwork();

  ASSERT_EQ(network.getCapacity(p, q), 1);
  ASSERT_EQ(network.getCapacity(q, p), 0);

  network.decrement(p, q);
  network.increment(q, p);
  ASSERT_EQ(network.getCapacity(p, q), 0);
  ASSERT_EQ(network.getCapacity(q, p), 1);

  network.setFlow(q, p);
  ASSERT_EQ(network.getCapacity(p, q), 1);
  ASSERT_EQ(network.getCapacity(q, p), 0);

  LibTEN::TEN_Node::clear();
}
