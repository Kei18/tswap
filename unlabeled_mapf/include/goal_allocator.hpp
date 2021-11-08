// target assignment with lazy evaluation

#pragma once
#include <queue>

#include "lib_ga.hpp"
#include "problem.hpp"

class GoalAllocator
{
public:
  enum MODE {
    BOTTLENECK_LINEAR,
    BOTTLENECK,
    LINEAR,
    GREEDY,
    GREEDY_SWAP
  };

private:
  Problem* P;
  Nodes assigned_goals;  // assignment results

  const MODE assignment_mode;

  // for bottleneck assignment
  const bool evaluate_all;  // evaluate all distance, default: false

  // qualities
  int matching_cost;  // estimation of sum of costs
  int matching_makespan;  // estimation of makspan

  // lazy evaluation
  std::vector<std::queue<Node*>> OPEN_LAZY;
  std::vector<std::vector<int>> DIST_LAZY;
  int getLazyEval(const int i, Node* const g);
  void setAllStartGoalDistances();  // compute all start-goal pairs of distance

  void bottleneckAssign();
  void linearAssign();
  void greedyAssign();
  void greedySwapAssign();

public:
  GoalAllocator(Problem* _P,
                MODE _mode = MODE::BOTTLENECK,
                bool _evaluate_all = false);
  ~GoalAllocator();

  // solve the problem
  void assign();

  // get results
  Nodes getAssignedGoals() const;
  int getMakespan() const;
  int getCost() const;
};
