#include <problem.hpp>
#include <ten_incremental.hpp>

#include "gtest/gtest.h"

TEST(TEN_INCREMENTAL, update)
{
  Problem P = Problem("../tests/instances/03.txt");
  auto network = TEN_INCREMENTAL(&P);

  network.update();
  ASSERT_EQ(network.getNodesNum(), 16);
  ASSERT_EQ(network.getEdgesNum(), 23);
  ASSERT_FALSE(network.isValid());

  network.update();
  ASSERT_EQ(network.getNodesNum(), 30);
  ASSERT_EQ(network.getEdgesNum(), 46);
  ASSERT_TRUE(network.isValid());
  ASSERT_EQ(network.getPlan().getSOC(), 3);
}
