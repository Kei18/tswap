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

    Matching(Problem *P);

    void mariage(const int s, const int g);

    void update(FieldEdge const *e);
  };

  struct MinCostMatching {
    const Nodes starts;
    const Nodes goals;
    const int N;
    const int NIL;

    std::vector<std::vector<int>> adj;
    std::vector<int> mate;

    std::vector<std::vector<int>> cost;

    int matched_num;
    Nodes assigned_goals;

    MinCostMatching(Problem *P)
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

    void mariage(const int s, const int g)
    {
      if (mate[s] == NIL) ++matched_num;
      mate[s] = g;
      mate[g] = s;
      assigned_goals[s] = goals[g - N];
    }

    void reset()
    {
      matched_num = 0;
      for (int i = 0; i < N*2; ++i) {
        if (i < N) assigned_goals[i] = nullptr;
        mate[i] = NIL;
      }
    }

    void addEdge(FieldEdge const *e)
    {
      int s = e->start_index;
      int g = N + e->goal_index;
      adj[s].push_back(g);
      adj[g].push_back(s);
      cost[s][g-N] = e->d;
    }

    void update()
    {
      reset();

      const int INF = 100000;
      const int SINK = N*2;
      std::vector<int> potential(N*2+1, 0);

      // while (true) {
      for (int i = 0; i < 3; ++i) {

        // agents + sink
        std::vector<int> dist(N*2+1, INF);
        std::vector<int> parent(N*2+1, NIL);

        auto compare = [&] (int v, int u) { return dist[v] > dist[u]; };
        std::priority_queue<int, std::vector<int>, decltype(compare)> OPEN(compare);
        std::vector<bool> CLOSE(N*2+1, false);

        // setup initial OPEN list, only for unmatched starts
        for (int v = 0; v < N; ++v) {
          if (mate[v] != NIL) continue;
          dist[v] = 0;
          OPEN.push(v);
        }

        // calculate distance by Dijkstra
        while (!OPEN.empty()) {
          // minimum node
          auto n = OPEN.top();
          OPEN.pop();

          // check CLOSE list
          if (CLOSE[n]) continue;
          CLOSE[n] = true;

          // sink
          if (n == SINK) continue;

          // expand neighbors
          for (auto m : adj[n]) {
            if (CLOSE[m]) continue;
            if (n <  N && mate[m] == n) continue;  // n: start -> m: goal
            if (n >= N && mate[m] != n) continue;  // n: goal  -> m: start
            // update distance, s->g or g->s
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

        // update potential & path
        for (int v = 0; v <= N*2; ++v) {
          if (dist[v] == INF) continue;
          potential[v] += dist[v];
        }

        if (parent[SINK] == NIL) {
          break;
        } else {
          // found path
          int n = parent[SINK];
          while (n != NIL) {
            mariage(parent[n], n);
            n = parent[parent[n]];
          }
          if (matched_num == N) break;  // maximum match
        }
      }
    }
  };
};
