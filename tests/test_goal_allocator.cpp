#include <goal_allocator.hpp>

#include "gtest/gtest.h"

TEST(GoalAllocator, test)
{
  Problem P = Problem("../tests/instances/05.txt");
  GoalAllocator allocator = GoalAllocator(&P);
  allocator.assign();

  Nodes assigned_goals = allocator.getAssignedGoals();
  ASSERT_FALSE(permutatedConfig(assigned_goals, P.getConfigStart()));
  ASSERT_TRUE(permutatedConfig(assigned_goals, P.getConfigGoal()));
  ASSERT_EQ(assigned_goals[0]->id, P.getG()->getNode(1,0)->id);
  ASSERT_EQ(assigned_goals[1]->id, P.getG()->getNode(3,0)->id);
}

TEST(GoalAllocator, large_field)
{
  Problem P = Problem("../instances/den520d_500agents_1.txt");
  GoalAllocator allocator = GoalAllocator(&P);
  allocator.assign();

  Nodes assigned_goals = allocator.getAssignedGoals();
  ASSERT_FALSE(permutatedConfig(assigned_goals, P.getConfigStart()));
  ASSERT_TRUE(permutatedConfig(assigned_goals, P.getConfigGoal()));
}
