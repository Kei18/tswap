#pragma once
#include "problem.hpp"
#include "lib_ga.hpp"

class GoalAllocator {
private:
  Problem* P;
  Nodes assigned_goals;

  const bool use_min_cost;
  int matching_cost;

public:
  GoalAllocator(Problem* _P, bool _use_min_cost=true);
  ~GoalAllocator();

  void assign();
  Nodes getAssignedGoals() const;
  int getCost() const;
};
