#include <problem.hpp>
#include <ten_incremental.hpp>

#include "gtest/gtest.h"

TEST(TEN_INCREMENTAL, update)
{
  Problem P = Problem("../tests/instances/03.txt");
  auto network = TEN_INCREMENTAL(&P);

  network.update();
  ASSERT_EQ(network.getNodesNum(), 10);
  ASSERT_EQ(network.getEdgesNum(), 14);
  ASSERT_FALSE(network.isValid());

  network.update();
  ASSERT_EQ(network.getNodesNum(), 18);
  ASSERT_EQ(network.getEdgesNum(), 28);
  ASSERT_TRUE(network.isValid());
  ASSERT_EQ(network.getPlan().getSOC(), 3);
}

TEST(TEN_INCREMENTAL, ILP)
{
  Problem P = Problem("../tests/instances/03.txt");
  auto network = TEN_INCREMENTAL(&P, false);

  network.update();
  ASSERT_FALSE(network.isValid());

  network.update();
  ASSERT_TRUE(network.isValid());
}
