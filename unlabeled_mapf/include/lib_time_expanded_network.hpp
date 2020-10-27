#pragma once
#include "problem.hpp"

// Node of TimeExpandedNetwork
struct TEN_Node;
using TEN_Nodes = std::vector<TEN_Node*>;


struct TEN_Node {
  enum NodeType { SOURCE, V_IN, V_OUT, W_IN, W_OUT, SINK };

  NodeType type;
  Node* v;  // original node
  Node* u;  // default nullptr, used when type \in { W_IN, W_OUT }, v->id < w->id
  int t;    // timestep
  std::string name;

  TEN_Nodes parents;
  TEN_Nodes children;

  static std::unordered_map<std::string, TEN_Node*> all_nodes;

  TEN_Node(NodeType _type, Node* _v, Node* _u, int _t);
  void addParent(TEN_Node* parent);
  void removeParent(TEN_Node* parent);

  static TEN_Node* createNewNode(NodeType _type, Node* _v, Node* _u, int _t);
  static TEN_Node* createNewNode(NodeType _type, Node* _v, int _t);
  static TEN_Node* createNewNode(NodeType _type);
  static std::string getName(NodeType _type, Node* _v, Node* _u, int _t);
  static std::string getName(NodeType _type, Node* _v, int _t);
  static std::string getEdgeName(TEN_Node* p, TEN_Node* q);
  static TEN_Node* getNode(NodeType _type, Node* _v, Node* _u, int _t);
  static TEN_Node* getNode(NodeType _type, Node* _v, int _t);
};
