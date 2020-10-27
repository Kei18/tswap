#pragma once
#include "plan.hpp"
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

  TEN_Node(NodeType _type, Node* _v, Node* _u, int _t)
    : type(_type), v(_v), u(_u), t(_t)
  {
    name = getName(type, _v, _u, t);
  }

  void addParent(TEN_Node* parent)
  {
    parents.push_back(parent);
    parent->children.push_back(this);
  }

  static TEN_Node* createNewNode(NodeType _type, Node* _v, Node* _u, int _t)
  {
    TEN_Node* new_node = new TEN_Node(_type, _v, _u, _t);
    all_nodes[new_node->name] = new_node;
    return new_node;
  }

  static TEN_Node* createNewNode(NodeType _type, Node* _v, int _t)
  {
    return createNewNode(_type, _v, nullptr, _t);
  }

  // used for sink or source
  static TEN_Node* createNewNode(NodeType _type)
  {
    if ((_type != NodeType::SOURCE) && (_type != NodeType::SINK)) halt("invalid type");
    return createNewNode(_type, nullptr, nullptr, 0);
  }

  static std::string getName(NodeType _type, Node* _v, Node* _u, int _t)
  {
    switch (_type) {
    case NodeType::SOURCE:
      return "source";
      break;
    case NodeType::SINK:
      return "sink";
      break;
    case NodeType::V_IN:
    case NodeType::V_OUT:
      return
        std::to_string(_v->id) + "_" +
        std::to_string(_t) + "_" +
        std::to_string(_type);
      break;
    case NodeType::W_IN:
    case NodeType::W_OUT:
      if (_u == nullptr) halt("invalid operation, TEN_Node::getName");
      return
        std::to_string(_v->id) + "_" +
        std::to_string(_u->id) + "_" +
        std::to_string(_t) + "_" +
        std::to_string(_type);
    default:
      halt("invalid operation, unknown TEN_Node type");
      return "";
    }
  }

  static std::string getName(NodeType _type, Node* _v, int _t)
  {
    return getName(_type, _v, nullptr, _t);
  }

  static std::string getEdgeName(TEN_Node* p, TEN_Node* q)
  {
    return p->name + "__" + q->name;
  }

  static TEN_Node* getNode(NodeType _type, Node* _v, Node* _u, int _t)
  {
    auto itr = all_nodes.find(getName(_type, _v, _u, _t));
    return (itr != all_nodes.end()) ? itr->second : nullptr;
  }

  static TEN_Node* getNode(NodeType _type, Node* _v, int _t)
  {
    return getNode(_type, _v, nullptr, _t);
  }
};


class TimeExpandedNetwork
{
private:
  Problem* P;   // original problem
  Nodes V;      // all nodes in G
  const int T;  // max timestep

  TEN_Node *source, *sink;
  std::unordered_map<std::string, int> residual_capacity;
  bool valid_network;
  Plan solution;

  void createGraph();
  void FordFulkerson();
  void createPlan();

public:
  TimeExpandedNetwork(Problem* _P, int _T);
  ~TimeExpandedNetwork();

  bool isValid() { return valid_network; }
  Plan getPlan() { return solution; }
};
