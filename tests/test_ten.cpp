#include <problem.hpp>
#include <ten.hpp>

#include "gtest/gtest.h"

TEST(TEN, update)
{
  Problem P = Problem("../tests/instances/03.txt");
  auto network1 = TEN(&P, 1);
  network1.update();
  ASSERT_EQ(network1.getNodesNum(), 10);
  ASSERT_EQ(network1.getEdgesNum(), 14);
  ASSERT_FALSE(network1.isValid());

  auto network2 = TEN(&P, 2);
  network2.update();
  ASSERT_EQ(network2.getNodesNum(), 18);
  ASSERT_EQ(network2.getEdgesNum(), 28);
  ASSERT_TRUE(network2.isValid());
  ASSERT_TRUE(network2.getPlan().validate(&P));
  ASSERT_EQ(network2.getPlan().getSOC(), 4);
}

TEST(TEN, dfs)
{
  Problem P = Problem("../tests/instances/04.txt");

  auto network1 = TEN(&P, 2);
  network1.update();
  ASSERT_EQ(network1.getDfsCnt(), 10 + 1);

  auto network2 = TEN(&P, 2, true);
  network2.update();
  ASSERT_EQ(network2.getDfsCnt(), 6 + 1);
}
