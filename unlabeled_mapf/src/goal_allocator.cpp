#include "../include/goal_allocator.hpp"

GoalAllocator::GoalAllocator(Problem* _P)
  : P(_P)
{
}

GoalAllocator::~GoalAllocator()
{
}

void GoalAllocator::assign()
{
  struct GA_Edge_Node {
    int start_index;
    int goal_index;
    Node* s;  // start
    Node* g;  // goal, target
    int d;    // distance

    GA_Edge_Node(int sindex, int gindex, Node* _s, Node* _g, int _d)
      : start_index(sindex), goal_index(gindex), s(_s), g(_g), d(_d) {}
  };
  using GA_Edge_Nodes = std::vector<GA_Edge_Node*>;

  auto compare = [](GA_Edge_Node* a, GA_Edge_Node* b) {
    if (a->d != b->d) return a->d > b->d;
    // tie break
    if (a->start_index != b->start_index) return a->start_index < b->start_index;
    return a->g->id < b->g->id;
  };


  GA_Edge_Nodes GC_EdgeNode;
  auto createNewEdgeNode = [&](int sindex, int gindex) {
    auto s = P->getStart(sindex);
    auto g = P->getGoal(gindex);
    auto p = new GA_Edge_Node(sindex, gindex, s, g, P->getG()->pathDist(s, g));
    GC_EdgeNode.push_back(p);
    return p;
  };

  std::priority_queue<GA_Edge_Node*, GA_Edge_Nodes, decltype(compare)> OPEN(compare);

  Config starts = P->getConfigStart();
  Config goals  = P->getConfigGoal();
  for (int i = 0; i < P->getNum(); ++i) {
    for (int j = 0; j < P->getNum(); ++j) OPEN.push(createNewEdgeNode(i, j));
  }


  // matching
  struct GA_Flow_Node;
  using GA_Flow_Nodes = std::vector<GA_Flow_Node*>;
  enum NodeType { SOURCE, START, GOAL, SINK };

  struct GA_Flow_Node {
    NodeType type;
    Node* v;
    GA_Flow_Nodes children;
    GA_Flow_Nodes parents;
    std::string name;

    GA_Flow_Node(NodeType _type, Node* _v)
      : type(_type), v(_v)
    {
      name = std::to_string(type) + ((v != nullptr) ? ("-" + std::to_string(v->id)) : "");
    };

    void addParent(GA_Flow_Node* parent)
    {
      parents.push_back(parent);
      parent->children.push_back(this);
    }
  };

  auto getEdgeName = [](GA_Flow_Node* a, GA_Flow_Node* b) { return a->name + "__" + b->name; };

  GA_Flow_Nodes GC_FlowNode;
  auto createNewFlowNode = [&](NodeType type, Node* v = nullptr) {
    auto p = new GA_Flow_Node(type, v);
    GC_FlowNode.push_back(p);
    return p;
  };

  // setup network
  std::unordered_map<std::string, int> capacity;
  auto source = createNewFlowNode(NodeType::SOURCE);
  auto sink   = createNewFlowNode(NodeType::SINK);
  GA_Flow_Nodes starts_flow;
  GA_Flow_Nodes goals_flow;
  for (auto v : starts) {
    auto p = createNewFlowNode(NodeType::START, v);
    p->addParent(source);
    starts_flow.push_back(p);
    capacity[getEdgeName(source, p)] = 1;
    capacity[getEdgeName(p, source)] = 0;
  }
  for (auto v : goals) {
    auto p = createNewFlowNode(NodeType::GOAL, v);
    sink->addParent(p);
    goals_flow.push_back(p);
    capacity[getEdgeName(p, sink)] = 1;
    capacity[getEdgeName(sink, p)] = 0;
  }

  while (!OPEN.empty()) {
    auto p = OPEN.top();
    OPEN.pop();

    // add edge
    auto parent = starts_flow[p->start_index];
    auto child  = goals_flow[p->goal_index];
    child->addParent(parent);
    capacity[getEdgeName(parent, child)] = 1;
    capacity[getEdgeName(child, parent)] = 0;

    // solving network flow
    while (true) {
      // depth first search
      std::unordered_map<GA_Flow_Node*, bool> CLOSED;
      auto dfs = [&](auto&& self, GA_Flow_Node* p) -> GA_Flow_Node* {
        // update closed list
        CLOSED[p] = true;

        // reach goal
        if (p == sink) return p;

        // check children
        GA_Flow_Nodes next;
        next.insert(next.end(), p->children.begin(), p->children.end());
        next.insert(next.end(), p->parents.begin(), p->parents.end());
        for (auto q : next) {
          // already searched
          if (CLOSED.find(q) != CLOSED.end()) continue;

          // fulfill
          if (capacity[getEdgeName(p, q)] == 0) continue;

          // recursive call
          auto res = self(self, q);

          // success
          if (res != nullptr) {
            capacity[getEdgeName(p, q)] -= 1;
            capacity[getEdgeName(q, p)] += 1;
            return res;
          }
        }

        // failed
        return nullptr;
      };

      if (dfs(dfs, source) == nullptr) break;
    }

    // check flow
    int flow = 0;
    for (auto s : starts_flow) flow += capacity[getEdgeName(s, source)];
    if (flow == P->getNum()) break;
  }

  // assign goals
  for (auto p : starts_flow) {
    for (auto q : p->children) {
      if (capacity[getEdgeName(q, p)] == 1) {
        assigned_goals.push_back(q->v);
      }
    }
  }

  // memory management
  for (auto p : GC_EdgeNode) delete p;
  for (auto p : GC_FlowNode) delete p;
}

Nodes GoalAllocator::getAssignedGoals() const
{
  return assigned_goals;
}
