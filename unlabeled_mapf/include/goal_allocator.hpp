#pragma once
#include "lib_ga.hpp"
#include "problem.hpp"

class GoalAllocator
{
private:
  Problem* P;
  Nodes assigned_goals;

  const bool use_bfs;
  const bool use_min_cost;
  int matching_cost;
  int matching_makespan;

  // void assignByBFS();

public:
  GoalAllocator(Problem* _P, bool _use_bfs=false, bool _use_min_cost=true);
  ~GoalAllocator();

  void assign();
  Nodes getAssignedGoals() const;
  int getMakespan() const;
  int getCost() const;
};
