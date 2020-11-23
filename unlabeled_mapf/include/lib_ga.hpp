#pragma once
#include <functional>

#include "graph.hpp"
#include "problem.hpp"

namespace LibGA
{
  struct FieldEdge;
  struct FlowNode;
  using FlowNodes = std::vector<FlowNode*>;

  struct FieldEdge {
    int start_index;
    int goal_index;
    Node* s;  // start
    Node* g;  // goal, target

    bool evaled;  // whether real distance is computed or not
    int inst_d;   // instance distance
    int d;        // real distance

    FieldEdge(int sindex, int gindex, Node* _s, Node* _g, int _d);

    void setRealDist(int _d);
  };

  struct Matching {
    const Nodes starts;
    const Nodes goals;
    const int N;
    const int NIL;
    std::vector<std::vector<int>> adj;
    std::vector<int> mate;
    std::vector<std::vector<int>> cost;  // start -> goal
    int matched_num;
    Nodes assigned_goals;

    Matching(Problem* P);

    void addEdge(FieldEdge const* e);
    void resetCurrentMate();
    void mariage(const int s, const int g);
    int getCost();

    void updateByIncrementalFordFulkerson(FieldEdge const* e);
    void solveBySuccessiveShortestPath();
  };
};  // namespace LibGA
