#include "../include/goal_allocator.hpp"

GoalAllocator::GoalAllocator(Problem* _P, bool _use_min_cost)
    : P(_P), use_min_cost(_use_min_cost), matching_cost(0), matching_makespan(0)
{
}

GoalAllocator::~GoalAllocator() {}

void GoalAllocator::assign()
{
  LibGA::OpenList OPEN(LibGA::FieldEdge::compare);

  // setup open list
  LibGA::FieldEdges GC_Edge;
  for (int i = 0; i < P->getNum(); ++i) {
    auto s = P->getStart(i);
    for (int j = 0; j < P->getNum(); ++j) {
      auto g = P->getGoal(j);
      auto p = new LibGA::FieldEdge(i, j, s, g, s->manhattanDist(g));
      GC_Edge.push_back(p);
      OPEN.push(p);
    }
  }

  auto matching = LibGA::Matching(P);

  while (!OPEN.empty()) {
    auto p = OPEN.top();
    OPEN.pop();

    // lazy evaluation
    if (!p->evaled) {
      p->setRealDist(P->getG()->pathDist(p->s, p->g));
      OPEN.push(p);
      continue;
    }

    matching.updateByIncrementalFordFulkerson(p);
    if (matching.matched_num == P->getNum()) {
      matching_makespan = p->d;
      break;
    }
  }

  // use min cost maximum matching
  if (use_min_cost) {
    matching.solveBySuccessiveShortestPath();
  }

  assigned_goals = matching.assigned_goals;
  matching_cost = matching.getCost();

  // memory management
  for (auto p : GC_Edge) delete p;
}

Nodes GoalAllocator::getAssignedGoals() const { return assigned_goals; }

int GoalAllocator::getCost() const { return matching_cost; }

int GoalAllocator::getMakespan() const { return matching_makespan; }
