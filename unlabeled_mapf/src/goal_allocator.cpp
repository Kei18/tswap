#include "../include/goal_allocator.hpp"

GoalAllocator::GoalAllocator(Problem* _P, bool _evaluate_all,
                             bool _use_min_cost, bool _use_greedy_assign)
    : P(_P),
      evaluate_all(_evaluate_all),
      use_min_cost(_use_min_cost),
      use_greedy_assign(_use_greedy_assign),
      matching_cost(0),
      matching_makespan(0),
      OPEN_LAZY(P->getNum()),
      DIST_LAZY(P->getNum(), std::vector<int>(P->getG()->getNodesSize(),
                                              P->getG()->getNodesSize()))
{
}

GoalAllocator::~GoalAllocator() {}

void GoalAllocator::assign()
{
  if (use_greedy_assign) {
    greedyAssign();
    return;
  }

  auto matching = LibGA::Matching(P);

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

    if (!evaluate_all) {
      // lazy evaluation
      for (int j = 0; j < P->getNum(); ++j) {
        auto g = P->getGoal(j);
        OPEN.emplace(i, j, s, g, s->manhattanDist(g));
      }
    } else {
      // without lazy eval
      OPEN_LAZY[i].push(s);
      DIST_LAZY[i][s->id] = 0;
      while (!OPEN_LAZY[i].empty()) {
        auto n = OPEN_LAZY[i].front();
        OPEN_LAZY[i].pop();
        const int d_n = DIST_LAZY[i][n->id];
        for (auto m : n->neighbor) {
          const int d_m = DIST_LAZY[i][m->id];
          if (d_n + 1 >= d_m) continue;
          DIST_LAZY[i][m->id] = d_n + 1;
          OPEN_LAZY[i].push(m);
        }
      }
      // set goal
      for (int j = 0; j < P->getNum(); ++j) {
        auto g = P->getGoal(j);
        OPEN.emplace(i, j, s, g, s->manhattanDist(g), DIST_LAZY[i][g->id]);
      }
    }
  }

  while (!OPEN.empty()) {
    auto p = OPEN.top();
    OPEN.pop();

    // lazy evaluation
    if (!p.evaled) {
      p.setRealDist(getLazyEval(p.start_index, p.g));
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

  // use min cost maximum matching
  if (use_min_cost) matching.solveBySuccessiveShortestPath();

  assigned_goals = matching.assigned_goals;
  matching_cost = matching.getCost();
}

int GoalAllocator::getLazyEval(const int i, Node* const g)
{
  // already evaluated
  if (DIST_LAZY[i][g->id] != P->getG()->getNodesSize())
    return DIST_LAZY[i][g->id];

  // initialize
  auto s = P->getStart(i);
  if (DIST_LAZY[i][s->id] != 0) {
    DIST_LAZY[i][s->id] = 0;
    OPEN_LAZY[i].push(s);
  }

  // BFS
  while (!OPEN_LAZY[i].empty()) {
    auto n = OPEN_LAZY[i].front();
    const int d_n = DIST_LAZY[i][n->id];

    // check goal condition
    if (n == g) return d_n;

    // pop
    OPEN_LAZY[i].pop();

    for (auto m : n->neighbor) {
      const int d_m = DIST_LAZY[i][m->id];
      if (d_n + 1 >= d_m) continue;
      DIST_LAZY[i][m->id] = d_n + 1;
      OPEN_LAZY[i].push(m);
    }
  }

  return P->getG()->getNodesSize();
}

Nodes GoalAllocator::getAssignedGoals() const { return assigned_goals; }

int GoalAllocator::getCost() const { return matching_cost; }

int GoalAllocator::getMakespan() const { return matching_makespan; }


// not used
void GoalAllocator::greedyAssignOnline()
{
  Nodes goals_availability(P->getG()->getNodesSize(), nullptr);
  for (auto g : P->getConfigGoal()) goals_availability[g->id] = g;

  // compute all distance
  for (int i = 0; i < P->getNum(); ++i) {
    auto s = P->getStart(i);
    OPEN_LAZY[i].push(s);
    DIST_LAZY[i][s->id] = 0;
    while (!OPEN_LAZY[i].empty()) {
      auto n = OPEN_LAZY[i].front();
      const int d_n = DIST_LAZY[i][n->id];

      // check goal condition
      if (goals_availability[n->id] != nullptr) {
        // assign
        assigned_goals.push_back(goals_availability[n->id]);
        goals_availability[n->id] = nullptr;
        matching_cost += d_n;
        matching_makespan = std::max(matching_makespan, d_n);
        break;
      }

      // pop
      OPEN_LAZY[i].pop();

      for (auto m : n->neighbor) {
        const int d_m = DIST_LAZY[i][m->id];
        if (d_n + 1 >= d_m) continue;
        DIST_LAZY[i][m->id] = d_n + 1;
        OPEN_LAZY[i].push(m);
      }
    }
  }
}


void GoalAllocator::greedyAssign()
{
  using Edge = std::tuple<int, Node*, int>;  // start, goal, distance
  std::vector<Edge> start_goal_pairs;
  auto compare = [&] (Edge a, Edge b) {
    if (std::get<2>(a) != std::get<2>(b)) return std::get<2>(a) < std::get<2>(b);
    if (std::get<0>(a) != std::get<0>(b)) return std::get<0>(a) < std::get<0>(b);
    if (std::get<1>(a) != std::get<1>(b)) return std::get<1>(a)->id < std::get<1>(b)->id;
    return false;
  };

  std::vector<bool> goal_indexes(P->getG()->getNodesSize(), false);
  for (auto g : P->getConfigGoal()) goal_indexes[g->id] = true;

  // compute all distance
  for (int i = 0; i < P->getNum(); ++i) {

    auto s = P->getStart(i);
    OPEN_LAZY[i].push(s);
    DIST_LAZY[i][s->id] = 0;

    int goal_cnt = 0;
    while (!OPEN_LAZY[i].empty()) {
      auto n = OPEN_LAZY[i].front();
      const int d_n = DIST_LAZY[i][n->id];

      // check goal condition
      if (goal_indexes[n->id]) {
        // insert
        start_goal_pairs.push_back(std::make_tuple(i, n, d_n));
        ++goal_cnt;
        // all distances are computed
        if (goal_cnt == P->getNum()) break;
      }

      // pop
      OPEN_LAZY[i].pop();

      for (auto m : n->neighbor) {
        const int d_m = DIST_LAZY[i][m->id];
        if (d_n + 1 >= d_m) continue;
        DIST_LAZY[i][m->id] = d_n + 1;
        OPEN_LAZY[i].push(m);
      }
    }
  }

  // sort
  sort(start_goal_pairs.begin(), start_goal_pairs.end(), compare);

  // initialize
  assigned_goals.clear();
  for (int i = 0; i < P->getNum(); ++i) assigned_goals.push_back(nullptr);
  matching_cost = 0;
  matching_makespan = 0;

  // greedy assign
  for (auto edge : start_goal_pairs) {
    auto index = std::get<0>(edge);
    auto g = std::get<1>(edge);
    auto d = std::get<2>(edge);

    // skip already assigned starts or goals
    if (assigned_goals[index] != nullptr) continue;
    if (!goal_indexes[g->id]) continue;

    // assign
    assigned_goals[index] = g;
    goal_indexes[g->id] = false;

    matching_cost += d;
    matching_makespan = std::max(matching_makespan, d);
  }
}
