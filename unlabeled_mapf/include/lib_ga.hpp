#pragma once
#include <functional>

#include "graph.hpp"
#include "problem.hpp"

namespace LibGA
{
  struct FieldEdge;
  struct FlowNode;
  using FlowNodes = std::vector<FlowNode*>;

  // pair of start and goal
  struct FieldEdge {
    int start_index;
    int goal_index;
    Node* s;  // start
    Node* g;  // goal

    bool evaled;  // whether real distance is computed or not
    int inst_d;   // estimated distance
    int d;        // real distance

    FieldEdge(int sindex, int gindex, Node* _s, Node* _g,
              int _d);  // with lazy eval
    FieldEdge(int sindex, int gindex, Node* _s, Node* _g, int _d1,
              int _d2);  // without lazy eval

    void setRealDist(int _d);
  };

  struct Matching {
    const Nodes starts;
    const Nodes goals;
    const int N;                         // number of starts
    static constexpr int NIL = -1;       // mean empty
    std::vector<std::vector<int>> adj;   // edges
    std::vector<int> mate;               // pair
    std::vector<std::vector<int>> cost;  // start -> goal
    int matched_num;
    Nodes assigned_goals;  // results

    Matching(Problem* P);

    void addEdge(FieldEdge const* e);
    void resetCurrentMate();
    void mariage(const int s, const int g);
    int getCost();

    // find one augmenting path
    void updateByIncrementalFordFulkerson(FieldEdge const* e);

    // successive shortest path algorithm
    void solveBySuccessiveShortestPath();
  };
};  // namespace LibGA
