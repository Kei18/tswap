// target assignment with lazy evaluation

#pragma once
#include "lib_ga.hpp"
#include "problem.hpp"

class GoalAllocator
{
private:
  Problem* P;
  Nodes assigned_goals;  // assignment

  const bool use_bfs;  // use BFS and compute all distances, default: false
  const bool
      use_min_cost;   // whether to use min-cost maximum matching, default: true
  int matching_cost;  // estimation of sum of costs
  int matching_makespan;  // estimation of makspan

public:
  GoalAllocator(Problem* _P, bool _use_bfs = false, bool _use_min_cost = true);
  ~GoalAllocator();

  // solve the problem
  void assign();

  // get results
  Nodes getAssignedGoals() const;
  int getMakespan() const;
  int getCost() const;
};
