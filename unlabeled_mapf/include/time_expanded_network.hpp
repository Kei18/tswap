#pragma once
#include "plan.hpp"
#include "problem.hpp"

class TimeExpandedNetwork
{
private:
  struct TEN_Node;
  using TEN_Nodes = std::vector<TEN_Node*>;
  enum NodeType { SOURCE, V_IN, V_OUT, W_IN, W_OUT, SINK };

  struct TEN_Node {
    TEN_Nodes parents;
    TEN_Nodes children;

    Node* v;
    Node* u;
    int t;
    NodeType type;
    std::string name;

    TEN_Node(Node* _v, int _t, NodeType _type, Node* _u = nullptr)
        : v(_v), u(_u), t(_t), type(_type)
    {
      name = getName(v, t, type, u);
      parents = {};
      children = {};
    }

    TEN_Node(NodeType _type) : v(nullptr), u(nullptr), t(0), type(_type)
    {
      name = getName(v, t, type, u);
      parents = {};
      children = {};
    }

    void addParent(TEN_Node* parent)
    {
      parents.push_back(parent);
      parent->children.push_back(this);
    }

    static std::string getName(Node* _v, int _t, NodeType _type,
                               Node* _u = nullptr)
    {
      if (_type == NodeType::SOURCE) {
        return "source";
      } else if (_type == NodeType::SINK) {
        return "sink";
      } else if (_u == nullptr) {
        return std::to_string(_v->id) + "_" + std::to_string(_t) + "_" +
               std::to_string(_type);
      } else {
        return std::to_string(_v->id) + "_" + std::to_string(_u->id) + "_" +
               std::to_string(_t) + "_" + std::to_string(_type);
      }
    }

    static std::string getEdgeName(TEN_Node* p, TEN_Node* q)
    {
      return p->name + "__" + q->name;
    }
  };

  Problem* P;
  Nodes V;
  const int T;  // max timestep

  TEN_Node *source, *sink;
  std::unordered_map<std::string, TEN_Node*> nodes;
  std::unordered_map<std::string, int> residual_capacity;
  bool valid_network;
  Plan plan;

  void createGraph();
  void FordFulkerson();
  void createPlan();

public:
  TimeExpandedNetwork(Problem* _P, int _T);
  ~TimeExpandedNetwork();

  bool isValid() { return valid_network; }
  Plan getPlan() { return plan; }
};
