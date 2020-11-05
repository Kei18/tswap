#pragma once
#include "graph.hpp"

namespace LibGA
{
  struct Edge;
  using Edges = std::vector<Edge*>;

  struct FlowNode;
  using FlowNodes = std::vector<FlowNode*>;


  struct Edge {
    int start_index;
    int goal_index;
    Node* s;  // start
    Node* g;  // goal, target

    bool evaled;  // whether real distance is computed or not
    int inst_d;   // instance distance
    int d;  // real distance

    Edge(int sindex, int gindex, Node* _s, Node* _g, int _d)
      : start_index(sindex),
        goal_index(gindex),
        s(_s),
        g(_g),
        evaled(false),
        inst_d(_d),
        d(0)
    {
    }

    void setRealDist(int _d)
    {
      if (!evaled) {
        evaled = true;
        d = _d;
      }
    }

    static bool compare(Edge* a, Edge* b)
    {
      if (a->evaled && b->evaled) {
        if (a->d != b->d) return a->d > b->d;
      } else if (!a->evaled && !b->evaled) {
        if (a->inst_d != b->inst_d) return a->inst_d > b->inst_d;
      } else if (a->evaled && !b->evaled) {
        if (a->d != b->inst_d) return a->d > b->inst_d;
      } else if (!a->evaled && b->evaled) {
        if (a->inst_d != b->d) return a->inst_d > b->d;
      }
      // tie break
      if (a->start_index != b->start_index) return a->start_index < b->start_index;
      return a->g->id < b->g->id;
    }
  };

  struct FlowNode {
    enum NodeType { SOURCE, START, GOAL, SINK };

    NodeType type;
    Node* v;
    FlowNodes children;
    FlowNodes parents;
    std::string name;

    FlowNode(NodeType _type, Node* _v)
      : type(_type), v(_v)
    {
      name = std::to_string(type) + ((v != nullptr) ? ("-" + std::to_string(v->id)) : "");
    };

    void addParent(FlowNode* parent)
    {
      parents.push_back(parent);
      parent->children.push_back(this);
    }
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

    Matching(Problem* P)
      : source(createNewNode(NodeType::SOURCE)),
        sink(createNewNode(NodeType::SINK)),
        start_cnt(0),
        goal_cnt(0)
    {
      for (auto v : P->getConfigStart()) createNewNode(NodeType::START, v);
      for (auto v : P->getConfigGoal())  createNewNode(NodeType::GOAL,  v);
    }

    ~Matching()
    {
      for (auto p : starts) delete p;
      for (auto p : goals) delete p;
      delete source;
      delete sink;
    }

    FlowNode* createNewNode(FlowNode::NodeType type, Node* v=nullptr)
    {
      auto p = new FlowNode(type, v);
      if (type == NodeType::START) {
        starts.push_back(p);
      } else if (type == NodeType::GOAL) {
        goals.push_back(p);
      }
      return p;
    }

    void addEdge(Edge* e)
    {
      if (e->start_index < 0 || starts.size() <= e->start_index) halt("FlowNode::addEdge, invalid");
      if (e->goal_index < 0  || goals.size()  <= e->goal_index)  halt("FlowNode::addEdge, invalid");

      auto parent = starts[e->start_index];
      auto child  = goals[e->goal_index];
      initEdge(parent, child);

      auto itr1 = unused_edge.find(getEdgeName(source, parent));
      if (itr1 == unused_edge.end()) {
        initEdge(source, parent);
        ++start_cnt;
      }

      auto itr2 = unused_edge.find(getEdgeName(child, sink));
      if (itr2 == unused_edge.end()) {
        initEdge(child, sink);
        ++goal_cnt;
      }
    }

    void initEdge(FlowNode* from, FlowNode* to)
    {
      to->addParent(from);
      use(to, from);
    }

    void use(FlowNode* from, FlowNode* to)
    {
      unused_edge[getEdgeName(from, to)] = false;
      unused_edge[getEdgeName(to, from)] = true;
    }

    bool unused(FlowNode* from, FlowNode* to) const
    {
      auto itr = unused_edge.find(getEdgeName(from, to));
      if (itr != unused_edge.end()) return itr->second;
      halt("unknown flow");
      return false;
    }

    static std::string getEdgeName(FlowNode* from, FlowNode* to)
    {
      return from->name + "__" + to->name;
    };

    void update()
    {
      while (true) {
        // depth first search
        std::unordered_map<LibGA::FlowNode*, bool> CLOSED;
        auto dfs = [&](auto&& self, FlowNode* p) -> FlowNode* {
          // update closed list
          CLOSED[p] = true;

          // reach goal
          if (p == sink) return p;

          // check children
          FlowNodes next;
          next.insert(next.end(), p->children.begin(), p->children.end());
          next.insert(next.end(), p->parents.begin(), p->parents.end());
          for (auto q : next) {
            // already searched
            if (CLOSED.find(q) != CLOSED.end()) continue;

            // used
            if (!unused(p, q)) continue;

            // recursive call
            auto res = self(self, q);

            // success
            if (res != nullptr) {
              use(p, q);
              return res;
            }
          }

          // failed
          return nullptr;
        };

        auto res = dfs(dfs, source);
        if (res == nullptr) break;
      }
    }

    int getMatchedNum()
    {
      return std::accumulate(starts.begin(), starts.end(), 0,
                             [&](int acc, decltype(starts)::value_type& itr) {
                               return acc + (int)unused(itr, source); });
    }

    bool matchedToSomeone(int index) const
    {
      if (index < 0 || starts.size() <= index) halt("FlowNode::addEdge, invalid");
      return !unused(source, starts[index]);
    }

    Nodes getAssignedGoals() const
    {
      Nodes assigned_goals;
      for (auto p : starts) {
        for (auto q : p->children) {
          if (!unused(p, q)) {
            assigned_goals.push_back(q->v);
            continue;
          }
        }
      }
      return assigned_goals;
    }
  };
};
