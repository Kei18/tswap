#include "../include/goal_allocator.hpp"

GoalAllocator::GoalAllocator(Problem* _P, bool _use_min_cost)
    : P(_P), use_min_cost(_use_min_cost), matching_cost(0), matching_makespan(0)
{
}

GoalAllocator::~GoalAllocator() {}

void GoalAllocator::assign()
{
  // setup priority queue
  auto compare = [](const LibGA::FieldEdge& a, const LibGA::FieldEdge& b) {
    if (a.evaled && b.evaled) {
      if (a.d != b.d) return a.d > b.d;
    } else if (!a.evaled && !b.evaled) {
      if (a.inst_d != b.inst_d) return a.inst_d > b.inst_d;
    } else if (a.evaled && !b.evaled) {
      if (a.d != b.inst_d) return a.d > b.inst_d;
    } else if (!a.evaled && b.evaled) {
      if (a.inst_d != b.d) return a.inst_d > b.d;
    }
    // tie break
    if (a.start_index != b.start_index) return a.start_index < b.start_index;
    return a.g->id < b.g->id;
  };

  // setup open list
  std::priority_queue<LibGA::FieldEdge,
                      std::vector<LibGA::FieldEdge>,
                      decltype(compare)> OPEN(compare);

  for (int i = 0; i < P->getNum(); ++i) {
    auto s = P->getStart(i);
    for (int j = 0; j < P->getNum(); ++j) {
      auto g = P->getGoal(j);
      OPEN.emplace(i, j, s, g, s->manhattanDist(g));
    }
  }

  auto matching = LibGA::Matching(P);

  while (!OPEN.empty()) {
    auto p = OPEN.top();
    OPEN.pop();

    // lazy evaluation
    if (!p.evaled) {
      p.setRealDist(P->getG()->pathDist(p.s, p.g));
      OPEN.push(p);
      continue;
    }

    matching.updateByIncrementalFordFulkerson(&p);

    if (matching.matched_num == P->getNum()) {
      matching_makespan = p.d;
      break;
    }
  }

  // use min cost maximum matching
  if (use_min_cost) {
    // add equal cost edges
    while (!OPEN.empty()) {
      auto p = OPEN.top();
      OPEN.pop();

      // lazy evaluation
      if (!p.evaled) {
        p.setRealDist(P->getG()->pathDist(p.s, p.g));
        OPEN.push(p);
        continue;
      }

      if (p.d > matching_makespan) break;
      matching.addEdge(&p);
    }

    matching.solveBySuccessiveShortestPath();
  }

  assigned_goals = matching.assigned_goals;
  matching_cost = matching.getCost();
}

Nodes GoalAllocator::getAssignedGoals() const { return assigned_goals; }

int GoalAllocator::getCost() const { return matching_cost; }

int GoalAllocator::getMakespan() const { return matching_makespan; }
