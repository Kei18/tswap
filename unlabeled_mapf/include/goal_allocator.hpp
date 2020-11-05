#pragma once
#include "problem.hpp"
#include "lib_ga.hpp"

class GoalAllocator {
private:
  Problem* P;
  Nodes assigned_goals;

public:
  GoalAllocator(Problem* P);
  ~GoalAllocator();

  void assign();
  Nodes getAssignedGoals() const;
};
