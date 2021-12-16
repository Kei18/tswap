#include "../include/lib_ga.hpp"

#include <climits>
#include <cstring>
#include <queue>

LibGA::FieldEdge::FieldEdge(int sindex, int gindex, Node* _s, Node* _g, int _d)
    : start_index(sindex),
      goal_index(gindex),
      s(_s),
      g(_g),
      evaled(false),
      inst_d(_d),
      d(0)
{
}

LibGA::FieldEdge::FieldEdge(int sindex, int gindex, Node* _s, Node* _g, int _d1,
                            int _d2)
    : start_index(sindex),
      goal_index(gindex),
      s(_s),
      g(_g),
      evaled(true),
      inst_d(_d1),
      d(_d2)
{
}

void LibGA::FieldEdge::setRealDist(int _d)
{
  if (!evaled) {
    evaled = true;
    d = _d;
  }
}

LibGA::Matching::Matching(Problem* P)
    : starts(P->getConfigStart()),
      goals(P->getConfigGoal()),
      N(P->getNum()),
      adj(N * 2, std::vector<int>()),
      mate(N * 2, NIL),
      cost(N, std::vector<int>(N, NIL)),
      matched_num(0),
      assigned_goals(N, nullptr)
{
}

void LibGA::Matching::addEdge(FieldEdge const* e)
{
  int s = e->start_index;
  int g = N + e->goal_index;
  adj[s].push_back(g);
  adj[g].push_back(s);
  cost[s][g - N] = e->d;
}

void LibGA::Matching::resetCurrentMate()
{
  matched_num = 0;
  for (int i = 0; i < N * 2; ++i) {
    if (i < N) assigned_goals[i] = nullptr;
    mate[i] = NIL;
  }
}

void LibGA::Matching::mariage(const int s, const int g)
{
  if (mate[s] == NIL) ++matched_num;
  mate[s] = g;
  mate[g] = s;
  assigned_goals[s] = goals[g - N];
}

int LibGA::Matching::getCost()
{
  int sum = 0;
  for (int i = 0; i < N; ++i) {
    if (mate[i] == NIL) continue;
    sum += cost[i][mate[i] - N];
  }
  return sum;
}

int LibGA::Matching::getMakespan()
{
  int score = 0;
  for (int i = 0; i < N; ++i) {
    if (mate[i] == NIL) continue;
    score = std::max(score, cost[i][mate[i] - N]);
  }
  return score;
}

void LibGA::Matching::updateByIncrementalFordFulkerson(FieldEdge const* e)
{
  addEdge(e);

  // close list
  std::vector<bool> CLOSE(N * 2, false);

  // DFS
  std::function<bool(int)> dfs = [&](int v) {  // start/goal
    if (CLOSE[v]) return false;
    CLOSE[v] = true;
    // expand neighbors
    for (int u : adj[v]) {  // u: goal/start
      int w = mate[u];      // w: start/goal
      // unmatched goal/start is found || found augmented path
      if (w == NIL || (!CLOSE[w] && dfs(w))) {
        if (v < N) {  // start
          mariage(v, u);
        } else {  // goal
          mariage(u, v);
        }
        return true;
      }
    }
    return false;
  };

  const int s = e->start_index;
  const int g = N + e->goal_index;
  if (mate[s] == NIL) {  // new path must include s
    dfs(s);
  } else if (mate[g] == NIL) {  // new path must include g
    dfs(g);
  } else {  // search all
    for (int v = 0; v < N; ++v) {
      if (mate[v] == NIL && dfs(v)) break;
    }
  }
}

void LibGA::Matching::solveBySuccessiveShortestPath()
{
  // clear the previous results
  resetCurrentMate();

  // setup sink node
  const int SINK = N * 2;

  // setup neighbors
  std::vector<int> adj_sink(N);
  std::iota(adj_sink.begin(), adj_sink.end(), N);
  adj.push_back(adj_sink);
  for (int v = N; v < N * 2; ++v) adj[v].push_back(SINK);
  std::vector<bool> f_to_sink(N, false);

  // potential
  std::vector<int> potential(N * 2 + 1, 0);

  struct DijkstraNode {
    int v;            // node
    int d;            // distance
    DijkstraNode* p;  // parent
  };
  using DijkstraNodes = std::vector<DijkstraNode*>;
  auto compare = [&](DijkstraNode* v, DijkstraNode* u) { return v->d > u->d; };

  // avoid "new" operation
  const int MEMORY_SIZE = N * 40;
  DijkstraNode GC[MEMORY_SIZE];

  for (int _i = 0; _i < N; ++_i) {
    // priority queue
    std::priority_queue<DijkstraNode*, DijkstraNodes, decltype(compare)> OPEN(
        compare);

    int node_total_cnt = 0;
    auto createNewNode = [&](int _v, int _d, DijkstraNode* _p) {
      if (node_total_cnt >= MEMORY_SIZE)
        halt("lib_ga, memory over, increase MEMORY_SIZE...");
      auto q = &(GC[node_total_cnt++]);
      q->v = _v;
      q->d = _d;
      q->p = _p;
      return q;
    };

    // close list
    bool CLOSE[N * 2 + 1];
    std::memset(CLOSE, false, sizeof(CLOSE));

    // distance from source
    std::vector<int> dist(N * 2 + 1, INT_MAX);

    // for backtracking
    DijkstraNode* sink_p = nullptr;

    // initialize
    for (int v = 0; v < N; ++v) {
      if (mate[v] != NIL) continue;
      dist[v] = 0;
      OPEN.push(createNewNode(v, 0, nullptr));
    }

    // calculate distance
    while (!OPEN.empty()) {
      // minimum node
      auto n = OPEN.top();
      OPEN.pop();

      // check CLOSE list
      if (CLOSE[n->v]) continue;
      CLOSE[n->v] = true;

      // expand neighbors
      for (auto m : adj[n->v]) {
        // already searched
        if (CLOSE[m]) continue;

        // check connectivity
        if (n->v < N) {  // start -> goal
          if (mate[n->v] == m) continue;
        } else if (n->v < N * 2 && m < N) {  // goal -> start
          if (mate[n->v] != m) continue;
        } else if (n->v < N * 2 && m == SINK) {  // goal -> sink
          if (f_to_sink[n->v - N]) continue;
        } else if (n->v == SINK) {  // sink -> goal
          if (!f_to_sink[m - N]) continue;
        } else {
          halt("unknown case");
        }

        // update distance, s -> g or g -> s
        int _c = (m == SINK || n->v == SINK)
                     ? 0
                     : ((n->v < N) ? cost[n->v][m - N] : -cost[m][n->v - N]);
        int c = _c + potential[n->v] - potential[m];
        int d = dist[n->v] + c;

        if (c < 0) halt("invalid cost: " + std::to_string(c));

        if (d < dist[m]) {
          dist[m] = d;
          auto p = createNewNode(m, d, n);
          OPEN.push(p);

          // for backtracking
          if (p->v == SINK) sink_p = p;
        }
      }
    }

    // update potential
    for (int v = 0; v < N * 2 + 1; ++v) {
      if (!CLOSE[v]) halt("unknown place: " + std::to_string(v));
      potential[v] += dist[v];
    }

    // backtracking
    if (sink_p != nullptr) {
      auto n = sink_p;
      while (n->p != nullptr) {
        if (n->v == SINK) {  // goal -> sink
          f_to_sink[n->p->v - N] = true;
        } else if (n->p->v == SINK) {   // sink -> goal
          f_to_sink[n->v - N] = false;  // meaningless
        } else if (n->v >= N) {         // start -> goal
          mariage(n->p->v, n->v);
        } else {  // goal -> start
          // pass
        }
        n = n->p;
      }
    }
  }
}
