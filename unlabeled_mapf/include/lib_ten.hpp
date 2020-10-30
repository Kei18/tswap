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
    enum NodeType { SOURCE, V_IN, V_OUT, W_IN, W_OUT, SINK };

    NodeType type;
    Node* v;  // original node
    Node* u;  // default nullptr, used when type \in { W_IN, W_OUT }, v->id <
              // w->id
    int t;    // timestep
    std::string name;

    TEN_Nodes parents;
    TEN_Nodes children;

    TEN_Node(NodeType _type, Node* _v, Node* _u, int _t);
    void addParent(TEN_Node* parent);
    void removeParent(TEN_Node* parent);

    static std::string getName(NodeType _type, Node* _v, Node* _u, int _t);
    static std::string getName(NodeType _type, Node* _v, int _t);

    std::string getStr();
  };

  struct ResidualNetwork {
    TEN_Node* source;
    TEN_Node* sink;
    std::unordered_map<std::string, TEN_Node*> body;
    std::unordered_map<std::string, int> capacity;

    const bool apply_filter;
    const Nodes goals;
    std::unordered_map<Node*, int> reachable_filter;

    int dfs_cnt;

    ResidualNetwork();
    ResidualNetwork(bool _filter, const Config& goals);
    ~ResidualNetwork();
    void init();

    using NodeType = TEN_Node::NodeType;

    TEN_Node* createNewNode(NodeType _type, Node* _v, Node* _u, int _t);
    TEN_Node* createNewNode(NodeType _type, Node* _v, int _t);
    TEN_Node* createNewNode(NodeType _type);

    TEN_Node* getNode(NodeType _type, Node* _v, Node* _u, int _t);
    TEN_Node* getNode(NodeType _type, Node* _v, int _t);

    static std::string getEdgeName(TEN_Node* p, TEN_Node* q);

    int getNodesNum();
    int getEdgesNum();

    int getCapacity(TEN_Node* p, TEN_Node* q);
    void initEdge(TEN_Node* p, TEN_Node* q);
    void deleteEdge(TEN_Node* p, TEN_Node* q);
    void increment(TEN_Node* p, TEN_Node* q);
    void decrement(TEN_Node* p, TEN_Node* q);
    void setFlow(TEN_Node* from, TEN_Node* to);

    void FordFulkerson();
    void createFilter(const Nodes& goals);

    int getFlowSum();
  };

};  // namespace LibTEN
