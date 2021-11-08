#include "../include/goal_allocator.hpp"

GoalAllocator::GoalAllocator(Problem* _P,
                             MODE _mode,
                             bool _evaluate_all,
                             bool _use_min_cost)
    : P(_P),
      assignment_mode(_mode),
      evaluate_all(_evaluate_all),
      use_min_cost(_use_min_cost),
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
  switch (assignment_mode) {
  case BOTTLENECK:
    bottleneckAssign();
    break;
  case LINEAR:
    linearAssign();
    break;
  case GREEDY:
    greedyAssign();
    break;
  case GREEDY_SWAP:
    greedySwapAssign();
    break;
  default:
    break;
  }
}

void GoalAllocator::bottleneckAssign()
{
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


  if (!evaluate_all) {
    // lazy evaluation
    for (int i = 0; i < P->getNum(); ++i) {
      auto s = P->getStart(i);
      for (int j = 0; j < P->getNum(); ++j) {
        auto g = P->getGoal(j);
        OPEN.emplace(i, j, s, g, s->manhattanDist(g));
      }
    }
  } else {
    // without lazy eval
    setAllStartGoalDistances();
    for (int i = 0; i < P->getNum(); ++i) {
      auto s = P->getStart(i);
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

void GoalAllocator::linearAssign()
{
  auto matching = LibGA::Matching(P);
  setAllStartGoalDistances();

  for (int i = 0; i < P->getNum(); ++i) {
    auto s = P->getStart(i);
    for (int j = 0; j < P->getNum(); ++j) {
      auto g = P->getGoal(j);
      auto e = LibGA::FieldEdge(i, j, s, g, s->manhattanDist(g), DIST_LAZY[i][g->id]);
      matching.addEdge(&e);
    }
  }

  // use min cost maximum matching
  matching.solveBySuccessiveShortestPath();

  assigned_goals = matching.assigned_goals;
  matching_cost = matching.getCost();
  matching_makespan =  matching.getMakespan();
}


void GoalAllocator::greedyAssign()
{
  setAllStartGoalDistances();

  // create start-goal paris
  using Edge = std::tuple<int, Node*, int>;  // start, goal, distance
  std::vector<Edge> start_goal_pairs;
  for (int i = 0; i < P->getNum(); ++i) {
    for (auto g : P->getConfigGoal()) {
      start_goal_pairs.push_back(std::make_tuple(i, g, DIST_LAZY[i][g->id]));
    }
  }

  // sort
  auto compare = [&] (Edge a, Edge b) {
    if (std::get<2>(a) != std::get<2>(b)) return std::get<2>(a) < std::get<2>(b);
    if (std::get<0>(a) != std::get<0>(b)) return std::get<0>(a) < std::get<0>(b);
    if (std::get<1>(a) != std::get<1>(b)) return std::get<1>(a)->id < std::get<1>(b)->id;
    return false;
  };
  sort(start_goal_pairs.begin(), start_goal_pairs.end(), compare);

  // initialize
  std::vector<bool> goal_indexes_assigned(P->getG()->getNodesSize(), false);
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
    if (goal_indexes_assigned[g->id]) continue;

    // assign
    assigned_goals[index] = g;
    goal_indexes_assigned[g->id] = true;

    matching_cost += d;
    matching_makespan = std::max(matching_makespan, d);
  }
}

void GoalAllocator::greedySwapAssign()
{
  // initialize
  std::queue<int> Q;
  for (int i = 0; i < P->getNum(); ++i) {
    Q.push(i);
    auto s = P->getStart(i);
    OPEN_LAZY[i].push(s);
    DIST_LAZY[i][s->id] = 0;
  }

  assigned_goals.clear();
  constexpr int NON_TARGET = -2;
  constexpr int FREE_TARGET = -1;
  std::vector<int> goal_agent_pairs(P->getG()->getNodesSize(), NON_TARGET);
  for (int i = 0; i < P->getNum(); ++i) {
    goal_agent_pairs[P->getGoal(i)->id] = FREE_TARGET;
    assigned_goals.push_back(nullptr);
  }

  while (!Q.empty()) {
    auto i = Q.front();
    Q.pop();

    while (!OPEN_LAZY[i].empty()) {
      auto n = OPEN_LAZY[i].front();
      auto d_n = DIST_LAZY[i][n->id];

      // check assignment
      auto j = goal_agent_pairs[n->id];
      if (j == FREE_TARGET) {  // free
        assigned_goals[i] = n;
        goal_agent_pairs[n->id] = i;
        break;
      } else if (j != NON_TARGET && d_n < DIST_LAZY[j][n->id]) {
        assigned_goals[i] = n;
        goal_agent_pairs[n->id] = i;
        assigned_goals[j] = nullptr;
        Q.push(j);
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

  // iterative refinement
  while (true) {
    int i = 0;  // bottleneck agent
    int d_i = 0;  // bottleneck cost
    for (int k = 0; k < P->getNum(); ++k) {
      auto d = getLazyEval(k, assigned_goals[k]);
      if (d > d_i) {
        i = k;
        d_i = d;
      }
    }
    auto s_i = P->getStart(i);
    auto g_i = assigned_goals[i];
    bool updated = false;

    for (int j = 0; j < P->getNum(); ++j) {
      if (j == i) continue;
      auto g_j = assigned_goals[j];
      auto cost_now  = std::max(d_i, getLazyEval(j, g_j));  // known distance
      // heuristic distance
      auto cost_estimated = std::max(s_i->manhattanDist(g_j), P->getStart(j)->manhattanDist(g_i));
      if (cost_estimated >= cost_now) continue;
      // real distance
      auto cost_swap = std::max(getLazyEval(i, g_j), getLazyEval(j, g_i));
      if (cost_swap < cost_now) {
        assigned_goals[i] = g_j;
        assigned_goals[j] = g_i;
        updated = true;
        break;
      }
    }

    if (!updated) break;
  }

  // compute quality
  matching_cost = 0;
  matching_makespan = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    auto c = DIST_LAZY[i][assigned_goals[i]->id];
    matching_makespan = std::max(matching_makespan, c);
    matching_cost += c;
  }
}

void GoalAllocator::setAllStartGoalDistances()
{
  // for constant time checking
  std::vector<bool> goal_indexes(P->getG()->getNodesSize(), false);
  for (auto g : P->getConfigGoal()) goal_indexes[g->id] = true;

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
        ++goal_cnt;
        // all distances are computed
        if (goal_cnt == P->getNum()) break;
      }

      // pop
      OPEN_LAZY[i].pop();

      // expand neighbors
      for (auto m : n->neighbor) {
        const int d_m = DIST_LAZY[i][m->id];
        if (d_n + 1 >= d_m) continue;
        DIST_LAZY[i][m->id] = d_n + 1;
        OPEN_LAZY[i].push(m);
      }
    }
  }
}


Nodes GoalAllocator::getAssignedGoals() const { return assigned_goals; }

int GoalAllocator::getCost() const { return matching_cost; }

int GoalAllocator::getMakespan() const { return matching_makespan; }
