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
};
