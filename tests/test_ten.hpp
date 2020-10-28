#pragma once
#include "gtest/gtest.h"
#include <problem.hpp>
#include <ten.hpp>

TEST(TEN, update)
{
  Problem* P = new Problem("../tests/instances/03.txt");
  auto network1 = TEN(P, 1);
  network1.update();
  ASSERT_EQ(network1.getNodesNum(), 16);
  ASSERT_EQ(network1.getEdgesNum(), 23);
  ASSERT_FALSE(network1.isValid());

  auto network2 = TEN(P, 2);
  network2.update();
  ASSERT_EQ(network2.getNodesNum(), 30);
  ASSERT_EQ(network2.getEdgesNum(), 46);
  ASSERT_TRUE(network2.isValid());
  ASSERT_EQ(network2.getPlan().getSOC(), 4);

  delete P;
}
