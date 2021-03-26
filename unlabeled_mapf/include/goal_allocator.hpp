// target assignment with lazy evaluation

#pragma once
#include "lib_ga.hpp"
#include "problem.hpp"
#include <queue>

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

  // lazy evaluation
  std::vector<std::queue<Node*>> OPEN_LAZY;
  std::vector<std::vector<int>> DIST_LAZY;
  int getLazyEval(const int i, Node* const g);

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
