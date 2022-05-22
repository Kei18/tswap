#include "../include/goal_allocator.hpp"

#include <unordered_map>

GoalAllocator::GoalAllocator(Problem* _P, MODE _mode)
    : P(_P),
      assignment_mode(_mode),
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
    case BOTTLENECK_LINEAR:
      bottleneckAssign();
      break;
    case BOTTLENECK_LINEAR_WO_LAZY:
      bottleneckAssign();
      break;
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
    case GREEDY_SWAP_WO_LAZY:
      greedySwapAssignWoLazy();
      break;
    case GREEDY_SWAP_COST:
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

  if (assignment_mode != MODE::BOTTLENECK_LINEAR_WO_LAZY) {
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
    for (int i = 0; i < P->getNum(); ++i) {
      auto s = P->getStart(i);
      for (int j = 0; j < P->getNum(); ++j) {
        auto g = P->getGoal(j);
        OPEN.emplace(i, j, s, g, s->manhattanDist(g), getLazyEval(s, j));
      }
    }
  }

  while (!OPEN.empty()) {
    auto p = OPEN.top();
    OPEN.pop();

    // lazy evaluation
    if (!p.evaled) {
      p.setRealDist(getLazyEval(p.s, p.goal_index));
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
  if (assignment_mode != BOTTLENECK) matching.solveBySuccessiveShortestPath();

  assigned_goals = matching.assigned_goals;
  matching_cost = matching.getCost();
}

int GoalAllocator::getLazyEval(const int start_index, const int goal_index)
{
  return getLazyEval(P->getStart(start_index), goal_index);
}

int GoalAllocator::getLazyEval(Node* const s, const int goal_index)
{
  auto g = P->getGoal(goal_index);

  // already evaluated
  if (DIST_LAZY[goal_index][s->id] != P->getG()->getNodesSize())
    return DIST_LAZY[goal_index][s->id];

  // initialize
  if (DIST_LAZY[goal_index][g->id] != 0) {
    DIST_LAZY[goal_index][g->id] = 0;
    OPEN_LAZY[goal_index].push(g);
  }

  // BFS
  while (!OPEN_LAZY[goal_index].empty()) {
    auto n = OPEN_LAZY[goal_index].front();
    const int d_n = DIST_LAZY[goal_index][n->id];

    // check goal condition
    if (n == s) return d_n;

    // pop
    OPEN_LAZY[goal_index].pop();

    for (auto m : n->neighbor) {
      const int d_m = DIST_LAZY[goal_index][m->id];
      if (d_n + 1 >= d_m) continue;
      DIST_LAZY[goal_index][m->id] = d_n + 1;
      OPEN_LAZY[goal_index].push(m);
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
      auto e = LibGA::FieldEdge(i, j, s, g, s->manhattanDist(g),
                                DIST_LAZY[i][g->id]);
      matching.addEdge(&e);
    }
  }

  // use min cost maximum matching
  matching.solveBySuccessiveShortestPath();

  assigned_goals = matching.assigned_goals;
  matching_cost = matching.getCost();
  matching_makespan = matching.getMakespan();
}

void GoalAllocator::greedyAssign()
{
  setAllStartGoalDistances();

  // create start-goal paris
  using Edge = std::tuple<int, Node*, int>;  // start, goal, distance
  std::vector<Edge> start_goal_pairs;
  for (int i = 0; i < P->getNum(); ++i) {
    for (int j = 0; j < P->getNum(); ++j) {
      start_goal_pairs.push_back(
          std::make_tuple(i, P->getGoal(j), getLazyEval(i, j)));
    }
  }

  // sort
  auto compare = [&](Edge a, Edge b) {
    if (std::get<2>(a) != std::get<2>(b))
      return std::get<2>(a) < std::get<2>(b);
    if (std::get<0>(a) != std::get<0>(b))
      return std::get<0>(a) < std::get<0>(b);
    if (std::get<1>(a) != std::get<1>(b))
      return std::get<1>(a)->id < std::get<1>(b)->id;
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
    auto g = P->getGoal(i);
    OPEN_LAZY[i].push(g);
    DIST_LAZY[i][g->id] = 0;
  }

  assigned_goals.clear();
  assigned_starts.clear();
  constexpr int NON_START = -2;
  constexpr int FREE_START = -1;
  std::vector<int> start_agent_pairs(P->getG()->getNodesSize(), NON_START);
  for (int i = 0; i < P->getNum(); ++i) {
    start_agent_pairs[P->getStart(i)->id] = FREE_START;
    assigned_starts.push_back(nullptr);
  }

  while (!Q.empty()) {
    auto i = Q.front();
    Q.pop();

    while (!OPEN_LAZY[i].empty()) {
      auto n = OPEN_LAZY[i].front();
      auto d_n = DIST_LAZY[i][n->id];

      // check assignment
      auto j = start_agent_pairs[n->id];
      if (j == FREE_START) {  // free
        assigned_starts[i] = n;
        start_agent_pairs[n->id] = i;
        break;
      } else if (j != NON_START && d_n < DIST_LAZY[j][n->id]) {
        assigned_starts[i] = n;
        start_agent_pairs[n->id] = i;
        assigned_starts[j] = nullptr;
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

  if (assignment_mode == GREEDY_SWAP) {
    greedyRefine();
  } else if (assignment_mode == GREEDY_SWAP_COST) {
    greedyRefineSOC();
  }
}

void GoalAllocator::greedyRefine()
{
  // iterative refinement
  while (true) {
    int i = 0;      // bottleneck agent
    int c_now = 0;  // bottleneck cost
    for (int k = 0; k < P->getNum(); ++k) {
      auto d = getLazyEval(assigned_starts[k], k);
      if (d > c_now) {
        i = k;
        c_now = d;
      }
    }
    auto s_i = assigned_starts[i];
    auto g_i = P->getGoal(i);
    bool updated = false;

    for (int j = 0; j < P->getNum(); ++j) {
      if (j == i) continue;
      auto s_j = assigned_starts[j];
      auto g_j = P->getGoal(j);
      // heuristic distance
      if (std::max(s_i->manhattanDist(g_j), s_j->manhattanDist(g_i)) >= c_now)
        continue;
      // real distance
      auto c_swap = std::max(getLazyEval(s_i, j), getLazyEval(s_j, i));
      if (c_swap < c_now) {
        assigned_starts[i] = s_j;
        assigned_starts[j] = s_i;
        updated = true;
        break;
      }
    }

    if (!updated) break;
  }

  // assigned_starts -> assigned goals
  std::unordered_map<Node*, Node*> tmp;  //
  for (int i = 0; i < P->getNum(); ++i) tmp[assigned_starts[i]] = P->getGoal(i);
  assigned_goals.clear();
  for (int i = 0; i < P->getNum(); ++i)
    assigned_goals.push_back(tmp[P->getStart(i)]);

  // compute quality
  matching_cost = 0;
  matching_makespan = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    auto c = getLazyEval(assigned_starts[i], i);
    matching_makespan = std::max(matching_makespan, c);
    matching_cost += c;
  }
}

void GoalAllocator::greedyRefineSOC()
{
  // iterative refinement
  while (true) {
    bool updated = false;
    for (int i = 0; i < P->getNum(); ++i)
      for (int j = i + 1; j < P->getNum(); ++j) {
        auto s_i = assigned_starts[i];
        auto s_j = assigned_starts[j];
        auto g_i = P->getGoal(i);
        auto g_j = P->getGoal(j);
        auto c_now = getLazyEval(s_i, i) + getLazyEval(s_j, j);
        // heuristic distance
        if (s_i->manhattanDist(g_j) + s_j->manhattanDist(g_i) >= c_now)
          continue;
        // real distance
        auto c_swap = getLazyEval(s_i, j) + getLazyEval(s_j, i);
        if (c_swap < c_now) {
          assigned_starts[i] = s_j;
          assigned_starts[j] = s_i;
          updated = true;
        }
      }
    if (!updated) break;
  }

  // assigned_starts -> assigned goals
  std::unordered_map<Node*, Node*> tmp;
  for (int i = 0; i < P->getNum(); ++i) tmp[assigned_starts[i]] = P->getGoal(i);
  assigned_goals.clear();
  for (int i = 0; i < P->getNum(); ++i)
    assigned_goals.push_back(tmp[P->getStart(i)]);

  // compute quality
  matching_cost = 0;
  matching_makespan = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    auto c = getLazyEval(assigned_starts[i], i);
    matching_makespan = std::max(matching_makespan, c);
    matching_cost += c;
  }
}

void GoalAllocator::greedySwapAssignWoLazy()
{
  assigned_starts = Nodes(P->getNum(), nullptr);
  setAllStartGoalDistances();
  std::queue<int> U;  // undecided
  std::vector<std::queue<Node*>> D(P->getNum(),
                                   std::queue<Node*>());  // distance
  for (int i = 0; i < P->getNum(); ++i) {
    U.push(i);
    auto compare = [&](Node* a, Node* b) {
      auto d_a = getLazyEval(a, i);
      auto d_b = getLazyEval(b, i);
      if (d_a != d_b) return d_a < d_b;
      return a->id < b->id;
    };
    auto tmp = P->getConfigStart();
    std::sort(tmp.begin(), tmp.end(), compare);
    for (auto s : tmp) D[i].push(s);
  }

  constexpr int NIL = -1;
  std::vector<int> A(P->getG()->getNodesSize(), NIL);  // start -> agent

  while (!U.empty()) {
    auto i = U.front();
    U.pop();
    while (!D[i].empty()) {
      auto s = D[i].front();
      D[i].pop();
      auto j = A[s->id];
      if (j == NIL) {
        assigned_starts[i] = s;
        A[s->id] = i;
        break;
      } else if (getLazyEval(s, i) < getLazyEval(s, j)) {
        assigned_starts[j] = nullptr;
        assigned_starts[i] = s;
        A[s->id] = i;
        U.push(j);
        break;
      }
    }
  }

  greedyRefine();
}

void GoalAllocator::setAllStartGoalDistances()
{
  // for constant time checking
  std::vector<bool> start_indexes(P->getG()->getNodesSize(), false);
  for (auto s : P->getConfigStart()) start_indexes[s->id] = true;

  for (int i = 0; i < P->getNum(); ++i) {
    auto g = P->getGoal(i);
    OPEN_LAZY[i].push(g);
    DIST_LAZY[i][g->id] = 0;

    int start_cnt = 0;
    while (!OPEN_LAZY[i].empty()) {
      auto n = OPEN_LAZY[i].front();
      const int d_n = DIST_LAZY[i][n->id];

      // check goal condition
      if (start_indexes[n->id]) {
        ++start_cnt;
        // all distances are computed
        if (start_cnt == P->getNum()) break;
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
