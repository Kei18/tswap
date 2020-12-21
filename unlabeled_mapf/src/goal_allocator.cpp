#include "../include/goal_allocator.hpp"

GoalAllocator::GoalAllocator(Problem* _P, bool _use_bfs, bool _use_min_cost)
    : P(_P),
      use_bfs(_use_bfs),
      use_min_cost(_use_min_cost),
      matching_cost(0),
      matching_makespan(0)
{
}

GoalAllocator::~GoalAllocator() {}

void GoalAllocator::assign()
{
  auto matching = LibGA::Matching(P);

  if (use_bfs) {
    // evaluate all start-goal pairs first

    // setup open list
    std::vector<LibGA::FieldEdge> OPEN;

    auto G = P->getG();
    auto goals = P->getConfigGoal();
    for (int i = 0; i < P->getNum(); ++i) {
      auto s = P->getStart(i);
      G->BFS(s, goals);
      for (int j = 0; j < P->getNum(); ++j) {
        auto g = P->getGoal(j);
        OPEN.emplace_back(i, j, s, g, s->manhattanDist(g), G->pathDist(s, g));
      }
    }

    // sort
    std::sort(
        OPEN.begin(), OPEN.end(),
        [&](LibGA::FieldEdge a, LibGA::FieldEdge b) { return a.d < b.d; });

    for (auto itr = OPEN.begin(); itr != OPEN.end(); ++itr) {
      auto p = *itr;
      matching.updateByIncrementalFordFulkerson(&p);

      // perfect matching
      if (matching.matched_num == P->getNum()) {
        matching_makespan = p.d;
        // add equal cost edges
        while (itr + 1 != OPEN.end() && (itr + 1)->d == p.d) {
          matching.addEdge(&(*(itr + 1)));
          ++itr;
        }
        break;
      }
    }

  } else {
    // use priority queue & lazy evaluation

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
    std::priority_queue<LibGA::FieldEdge, std::vector<LibGA::FieldEdge>,
                        decltype(compare)>
        OPEN(compare);

    for (int i = 0; i < P->getNum(); ++i) {
      auto s = P->getStart(i);
      for (int j = 0; j < P->getNum(); ++j) {
        auto g = P->getGoal(j);
        OPEN.emplace(i, j, s, g, s->manhattanDist(g));
      }
    }

    while (!OPEN.empty()) {
      auto p = OPEN.top();
      OPEN.pop();

      // lazy evaluation
      if (!p.evaled) {
        p.setRealDist(P->getG()->pathDist(p.s, p.g));
        OPEN.push(p);
        continue;
      }

      if (matching_makespan > 0) {  // add equal cost edges
        if (p.d <= matching_makespan) {
          matching.addEdge(&p);
          continue;
        } else {
          break;  // end
        }

      } else {
        // update matching
        matching.updateByIncrementalFordFulkerson(&p);
      }

      // perfect match
      if (matching.matched_num == P->getNum()) matching_makespan = p.d;
    }
  }

  // use min cost maximum matching
  if (use_min_cost) matching.solveBySuccessiveShortestPath();

  assigned_goals = matching.assigned_goals;
  matching_cost = matching.getCost();
}

Nodes GoalAllocator::getAssignedGoals() const { return assigned_goals; }

int GoalAllocator::getCost() const { return matching_cost; }

int GoalAllocator::getMakespan() const { return matching_makespan; }
