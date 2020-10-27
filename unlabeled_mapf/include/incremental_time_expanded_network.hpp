#pragma once
#include "plan.hpp"
#include "problem.hpp"
#include "lib_time_expanded_network.hpp"

class IncrementalTimeExpandedNetwork {
  Problem* P;   // original problem
  Nodes V;      // all nodes in G
  int T;        // max timestep

  TEN_Node *source, *sink;
  std::unordered_map<std::string, int> residual_capacity;
  bool valid_network;
  Plan solution;

  void updateGraph();
  void FordFulkerson();
  void checkFlow();
  void createPlan();

  int getResidualCapacity(TEN_Node* p, TEN_Node* q);
  void initResidualCapacity(TEN_Node* p, TEN_Node* q);
  void incrementResidualCapacity(TEN_Node* p, TEN_Node* q);
  void decrementResidualCapacity(TEN_Node* p, TEN_Node* q);

public:
  IncrementalTimeExpandedNetwork(Problem* _P);
  ~IncrementalTimeExpandedNetwork();

  void update();

  bool isValid() { return valid_network; }
  Plan getPlan() { return solution; }

  int nodesNum();
  int edgesNum();
};
