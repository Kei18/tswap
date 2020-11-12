#pragma once
#include "graph.hpp"
#include "problem.hpp"
#include <functional>

namespace LibGA
{
  struct FieldEdge;
  using FieldEdges = std::vector<FieldEdge*>;
  using OpenList = std::priority_queue<FieldEdge*, FieldEdges,
                                       std::function<bool(FieldEdge*, FieldEdge*)>>;
  struct FlowNode;
  using FlowNodes = std::vector<FlowNode*>;


  struct FieldEdge {
    int start_index;
    int goal_index;
    Node* s;  // start
    Node* g;  // goal, target

    bool evaled;  // whether real distance is computed or not
    int inst_d;   // instance distance
    int d;  // real distance

    FieldEdge(int sindex, int gindex, Node* _s, Node* _g, int _d);

    void setRealDist(int _d);

    static bool compare(FieldEdge* a, FieldEdge* b);
  };

  struct Matching {
    const Nodes starts;
    const Nodes goals;
    const int N;
    const int NIL;
    std::vector<std::vector<int>> adj;
    std::vector<int> mate;
    int matched_num;
    Nodes assigned_goals;

    Matching(Problem* P)
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

    void mariage(int s, int g)
    {
      if (mate[s] == NIL) ++matched_num;
      mate[s] = g;
      mate[g] = s;
      assigned_goals[s] = goals[g - N];
    }

    void update(FieldEdge* e)
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
  };
};
