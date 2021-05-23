/*
 * This mainly contains utilities of Time Expanded Network (TEN) based solvers.
 */

#pragma once
#include "problem.hpp"

namespace LibTEN
{
  struct TEN_Node;
  using TEN_Nodes = std::vector<TEN_Node*>;

  struct TEN_Node {
    enum NodeType { SOURCE, V_IN, V_OUT, SINK };
    NodeType type;

    Node* v;  // original node
    int t;    // timestep

    // structure
    TEN_Nodes parents;
    TEN_Nodes children;

    // capacity, key: child
    std::unordered_map<TEN_Node*, bool> capacity;

    TEN_Node(NodeType _type, Node* _v, int _t);

    // operations to change structure
    void addParent(TEN_Node* parent);
    void removeParent(TEN_Node* parent);

    static std::string getName(NodeType _type, Node* _v, int _t);
  };

  struct ResidualNetwork {
    TEN_Node* source;
    TEN_Node* sink;

    std::vector<std::vector<TEN_Node*>> body_V_IN;   // store vertices (V_IN)
    std::vector<std::vector<TEN_Node*>> body_V_OUT;  // store vertices (V_OUT)

    const bool apply_filter;  // whether to prune redundant vertices
    Problem* P;

    // for pruning, <Node, required timestep to reach the sink>
    std::unordered_map<Node*, int> reachable_filter;

    int time_limit;

    // for FordFulkerson, count the number of visited nodes
    int dfs_cnt;

    ResidualNetwork();
    ResidualNetwork(bool _filter, Problem* _P);
    ~ResidualNetwork();
    void init();

    void setTimeLimit(int _time_limit) { time_limit = _time_limit; }

    using NodeType = TEN_Node::NodeType;

    TEN_Node* createNewNode(NodeType _type, Node* _v, int _t);
    TEN_Node* createNewNode(NodeType _type);

    TEN_Node* getNode(NodeType _type, Node* _v, Node* _u, int _t);
    TEN_Node* getNode(NodeType _type, Node* _v, int _t);

    // return size of network
    int getNodesNum();
    int getEdgesNum();

    int getCapacity(TEN_Node* p, TEN_Node* q);
    void clearAllCapacity();
    void deleteEdge(TEN_Node* p, TEN_Node* q);

    // judge used or not
    bool used(TEN_Node* p, TEN_Node* q);

    // update flow
    void setFlow(TEN_Node* from, TEN_Node* to);

    // update network structure
    void addParent(TEN_Node* child, TEN_Node* parent);
    void removeParent(TEN_Node* child, TEN_Node* parent);

    // solve the maximum flow problem
    void solve();
    void FordFulkerson();
    void FordFulkersonWithStack();
    void FordFulkersonWithRecursiveCall();
    void createFilter();

    // return maximum flow size
    int getFlowSum();
  };
};  // namespace LibTEN
