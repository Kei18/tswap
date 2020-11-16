#include "../include/lib_ga.hpp"

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

void LibGA::FieldEdge::setRealDist(int _d)
{
  if (!evaled) {
    evaled = true;
    d = _d;
  }
}

bool LibGA::FieldEdge::compare(FieldEdge* a, FieldEdge* b)
{
  if (a->evaled && b->evaled) {
    if (a->d != b->d) return a->d > b->d;
  } else if (!a->evaled && !b->evaled) {
    if (a->inst_d != b->inst_d) return a->inst_d > b->inst_d;
  } else if (a->evaled && !b->evaled) {
    if (a->d != b->inst_d) return a->d > b->inst_d;
  } else if (!a->evaled && b->evaled) {
    if (a->inst_d != b->d) return a->inst_d > b->d;
  }
  // tie break
  if (a->start_index != b->start_index) return a->start_index < b->start_index;
  return a->g->id < b->g->id;
}

LibGA::Matching::Matching(Problem *P)
  : starts(P->getConfigStart()),
    goals(P->getConfigGoal()),
    N(P->getNum()),
    NIL(N*2+1),
    adj(N*2),
    mate(N*2, NIL),
    cost(N, std::vector<int>(N, NIL)),
    matched_num(0),
    assigned_goals(N, nullptr)
{
}

void LibGA::Matching::addEdge(FieldEdge const *e)
{
  int s = e->start_index;
  int g = N + e->goal_index;
  adj[s].push_back(g);
  adj[g].push_back(s);
  cost[s][g-N] = e->d;
}

void LibGA::Matching::resetCurrentMate()
{
  matched_num = 0;
  for (int i = 0; i < N*2; ++i) {
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

void LibGA::Matching::updateByIncrementalFordFulkerson(FieldEdge const *e)
{
  addEdge(e);

  std::vector<bool> visited(N*2, false);
  std::function<bool(int)> dfs = [&](int v) {   // start/goal
    if (visited[v]) return false;
    visited[v] = true;
    // expand neighbors
    for (int u : adj[v]) {  // u: goal/start
      int w = mate[u];      // w: start/goal
      // unmatched goal/start is found || found augmented path
      if (w == NIL || (!visited[w] && dfs(w))) {
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
  if (mate[s] == NIL) {
    dfs(s);
  } else if (mate[g] == NIL) {
    dfs(g);
  } else {
    for (int v = 0; v < N; ++v) {
      if (mate[v] == NIL && dfs(v)) break;
    }
  }
}

void LibGA::Matching::solveBySuccessiveShortestPath()
{
  resetCurrentMate();

  const int INF = 100000;
  const int SINK = N*2;
  std::vector<int> potential(N*2+1, 0);

  while (true) {
    // agents + sink
    std::vector<int> dist(N*2+1, INF);
    std::vector<int> parent(N*2+1, NIL);

    // priority queue
    auto compare = [&] (int v, int u) { return dist[v] > dist[u]; };
    std::priority_queue<int, std::vector<int>, decltype(compare)> OPEN(compare);

    // close list
    std::vector<bool> CLOSE(N*2+1, false);

    // setup OPEN list, only for unmatched starts (avoid using source node)
    for (int v = 0; v < N; ++v) {
      if (mate[v] != NIL) continue;
      dist[v] = 0;
      OPEN.push(v);
    }

    // calculate distance/parent by Dijkstra
    while (!OPEN.empty()) {
      // minimum node
      auto n = OPEN.top();
      OPEN.pop();

      // check CLOSE list
      if (CLOSE[n]) continue;
      CLOSE[n] = true;

      // special case, sink
      if (n == SINK) continue;

      // expand neighbors
      for (auto m : adj[n]) {
        if (CLOSE[m]) continue;
        if (n <  N && mate[m] == n) continue;  // n: start -> m: goal
        if (n >= N && mate[m] != n) continue;  // n: goal  -> m: start
        // update distance, s -> g or g -> s
        int d = dist[n] + (((n < N) ? cost[n][m-N] : -cost[m][n-N]) + potential[n] - potential[m]);
        if (d < dist[m]) {
          dist[m] = d;
          parent[m] = n;
          OPEN.push(m);
        }
      }

      // check sink
      if (n >= N && !CLOSE[SINK] && mate[n] == NIL) {
        int d = dist[n] + 0 + potential[n] - potential[SINK];
        if (d < dist[SINK]) {
          dist[SINK] = d;
          parent[SINK] = n;
          OPEN.push(SINK);
        }
      }
    }

    // update potential
    for (int v = 0; v <= N*2; ++v) {
      if (dist[v] == INF) continue;
      potential[v] += dist[v];
    }

    if (parent[SINK] == NIL) {  // no path is found
      break;
    } else {  // found path
      int n = parent[SINK];
      while (n != NIL) {
        mariage(parent[n], n);
        n = parent[parent[n]];
      }
      if (matched_num == N) break;  // maximum match
    }
  }
}
