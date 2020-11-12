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
    matched_num(0),
    assigned_goals(N, nullptr)
{
}

void LibGA::Matching::mariage (const int s, const int g)
{
  if (mate[s] == NIL) ++matched_num;
  mate[s] = g;
  mate[g] = s;
  assigned_goals[s] = goals[g - N];
}

void LibGA::Matching::update(FieldEdge const *e)
{
  // add new edge
  int s = e->start_index;
  int g = N + e->goal_index;
  adj[s].push_back(g);
  adj[g].push_back(s);

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
