#include <problem.hpp>
#include <ten.hpp>

#include "gtest/gtest.h"

TEST(TEN, update)
{
  Problem P = Problem("../tests/instances/03.txt");
  auto network1 = TEN(&P, 1);
  network1.update();
  ASSERT_EQ(network1.getNodesNum(), 16);
  ASSERT_EQ(network1.getEdgesNum(), 23);
  ASSERT_FALSE(network1.isValid());

  auto network2 = TEN(&P, 2);
  network2.update();
  ASSERT_EQ(network2.getNodesNum(), 30);
  ASSERT_EQ(network2.getEdgesNum(), 46);
  ASSERT_TRUE(network2.isValid());
  ASSERT_EQ(network2.getPlan().getSOC(), 4);
}

TEST(TEN, dfs)
{
  Problem P = Problem("../tests/instances/04.txt");

  auto network1 = TEN(&P, 2);
  network1.update();
  ASSERT_EQ(network1.getDfsCnt(), 16 + 1);

  auto network2 = TEN(&P, 2, true);
  network2.update();
  ASSERT_EQ(network2.getDfsCnt(), 10 + 1);
}

TEST(TEN, ILP)
{
  Problem P = Problem("../tests/instances/03.txt");
  auto network = TEN(&P, 2, false, true);
  network.update();
  auto plan = network.getPlan();

  ASSERT_TRUE(network.isValid());
  ASSERT_TRUE(plan.validate(&P));
  ASSERT_TRUE(plan.getSOC() >= 3);
}
