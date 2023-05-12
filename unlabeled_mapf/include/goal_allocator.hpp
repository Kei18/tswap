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
    BOTTLENECK_LINEAR_WO_LAZY,
    BOTTLENECK,
    LINEAR,
    GREEDY,
    GREEDY_SWAP,
    GREEDY_SWAP_WO_LAZY,
    GREEDY_SWAP_COST,
  };

private:
  Problem* P;
  Nodes assigned_goals;   // start index -> goal
  Nodes assigned_starts;  // goal index -> start

  const MODE assignment_mode;

  // qualities
  int matching_cost;      // estimation of sum of costs
  int matching_makespan;  // estimation of makspan

  // lazy evaluation
  std::vector<std::queue<Node*>> OPEN_LAZY;
  std::vector<std::vector<int>> DIST_LAZY;

public:
  int getLazyEval(const int start_index, const int goal_index);
  int getLazyEval(Node* const s, const int goal_index);

private:
  void setAllStartGoalDistances();  // compute all start-goal pairs of distance

  void bottleneckAssign();
  void linearAssign();
  void greedyAssign();
  void greedySwapAssign();
  void greedySwapAssignWoLazy();
  void greedyRefine();
  void greedyRefineSOC();

public:
  GoalAllocator(Problem* _P, MODE _mode = MODE::BOTTLENECK_LINEAR);
  ~GoalAllocator();

  // solve the problem
  void assign();

  // get results
  Nodes getAssignedGoals() const;
  int getMakespan() const;
  int getCost() const;
};
