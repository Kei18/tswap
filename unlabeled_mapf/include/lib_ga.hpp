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

  struct FlowNode {
    enum NodeType { SOURCE, START, GOAL, SINK };

    NodeType type;
    Node* v;
    FlowNodes children;
    FlowNodes parents;
    std::string name;

    FlowNode(NodeType _type, Node* _v);

    void addParent(FlowNode* parent);
  };

  struct Matching {
    FlowNode* source;
    FlowNode* sink;
    LibGA::FlowNodes starts;
    LibGA::FlowNodes goals;
    std::unordered_map<std::string, bool> unused_edge;

    int start_cnt;
    int goal_cnt;

    using NodeType = FlowNode::NodeType;

    Matching(Problem* P);
    ~Matching();

    FlowNode* createNewNode(FlowNode::NodeType type, Node* v=nullptr);
    void addEdge(FieldEdge* e);
    void initEdge(FlowNode* from, FlowNode* to);
    void use(FlowNode* from, FlowNode* to);
    bool unused(FlowNode* from, FlowNode* to) const;
    static std::string getEdgeName(FlowNode* from, FlowNode* to);
    void update();
    int getMatchedNum();
    bool matchedToSomeone(int index) const;
    Nodes getAssignedGoals() const;
  };
};
