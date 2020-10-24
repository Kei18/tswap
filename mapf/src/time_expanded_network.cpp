#include "../include/time_expanded_network.hpp"
#include <unordered_map>
#include <stack>
#include <string>

TimeExpandedNetwork::TimeExpandedNetwork(Problem* _P, int _T)
  : P(_P), V(P->getG()->getV()), T(_T)
{
  valid_network = false;
  createGraph();
  FordFulkerson();
  createPlan();
}

TimeExpandedNetwork::~TimeExpandedNetwork()
{
  // free
  for (auto itr = nodes.begin(); itr != nodes.end(); ++itr) {
    delete itr->second;
  }
}

void TimeExpandedNetwork::createGraph()
{
  auto createNewNode =
    [&] (NodeType type, Node* v = nullptr, int t = 0, Node* u = nullptr)
    {
      TEN_Node* new_node;
      if (type == NodeType::SOURCE || type == NodeType::SINK) {
        new_node = new TEN_Node(type);
      } else {
        new_node = new TEN_Node(v, t, type, u);
      }
      nodes[new_node->name] = new_node;
      return new_node;
    };

  // add vertex
  for (int t = 1; t <= T; ++t) {
    for (auto v : V) {
      auto v_in  = createNewNode(NodeType::V_IN,  v, t);
      auto v_out = createNewNode(NodeType::V_OUT, v, t);

      v_out->addParent(v_in);
      if (t > 1) v_in->addParent(nodes[TEN_Node::getName(v, t-1, NodeType::V_OUT)]);
    }
  }

  // add edges
  for (int t = 1; t <= T; ++t) {
    for (auto v : V) {
      for (auto u : v->neighbor) {
        if (v->id >= u->id) continue;
        auto w_in  = createNewNode(NodeType::W_IN,  v, t, u);
        auto w_out = createNewNode(NodeType::W_OUT, v, t, u);
        w_in->addParent(nodes[TEN_Node::getName(v, t, NodeType::V_IN)]);
        w_in->addParent(nodes[TEN_Node::getName(u, t, NodeType::V_IN)]);
        w_out->addParent(w_in);
        nodes[TEN_Node::getName(v, t, NodeType::V_OUT)]->addParent(w_out);
        nodes[TEN_Node::getName(u, t, NodeType::V_OUT)]->addParent(w_out);
      }
    }
  }

  // add source
  source = createNewNode(NodeType::SOURCE);
  for (auto v : P->getConfigStart())
    nodes[TEN_Node::getName(v, 1, NodeType::V_IN)]->addParent(source);

  // add sink
  sink = createNewNode(NodeType::SINK);
  for (auto v : P->getConfigGoal())
    sink->addParent(nodes[TEN_Node::getName(v, T, NodeType::V_OUT)]);
}

void TimeExpandedNetwork::FordFulkerson()
{
  // initialize
  for (auto itr = nodes.begin(); itr != nodes.end(); ++itr) {
    auto p = itr->second;
    for (auto q : p->children) {
      residual_capacity[TEN_Node::getEdgeName(p, q)] = 1;
      residual_capacity[TEN_Node::getEdgeName(q, p)] = 0;
    }
  }

  while (true) {

    // depth first search
    std::unordered_map<std::string, bool> CLOSED;
    auto dfs = [&] (auto&& self, TEN_Node* p) -> TEN_Node*
    {
      CLOSED[p->name] = true;
      if (p == sink) return p;

      // check children
      TEN_Nodes next;
      next.insert(next.end(), p->children.begin(), p->children.end());
      next.insert(next.end(), p->parents.begin(), p->parents.end());
      for (auto q : next) {
        if (CLOSED.find(q->name) != CLOSED.end()) continue;
        if (residual_capacity[TEN_Node::getEdgeName(p, q)] == 0) continue;
        auto res = self(self, q);
        if (res != nullptr) {
          residual_capacity[TEN_Node::getEdgeName(p, q)] -= 1;
          residual_capacity[TEN_Node::getEdgeName(q, p)] += 1;
          return res;
        }
      }
      return nullptr;
    };

    if (dfs(dfs, source) == nullptr) break;
  }

  // check validity
  int flow = 0;
  for (auto p : source->children) {
    flow += residual_capacity[TEN_Node::getEdgeName(p, source)];
  }
  valid_network = (flow == P->getNum());
}

void TimeExpandedNetwork::createPlan()
{
  if (!valid_network) return;

  Config C = P->getConfigStart();
  Config C_next;
  plan.add(C);

  for (int t = 1; t <= T; ++t) {
    for (auto v : C) {
      Node* next_node = nullptr;
      // move
      for (auto u : v->neighbor) {
        auto first  = (v->id < u->id) ? v : u;
        auto second = (v->id < u->id) ? u : v;
        auto p1 = nodes[TEN_Node::getName(v, t, NodeType::V_IN)];
        auto q1 = nodes[TEN_Node::getName(first, t, NodeType::W_IN, second)];
        auto p2 = nodes[TEN_Node::getName(first, t, NodeType::W_OUT, second)];
        auto q2 = nodes[TEN_Node::getName(u, t, NodeType::V_OUT)];
        if (residual_capacity[TEN_Node::getEdgeName(p1, q1)] == 0 &&
            residual_capacity[TEN_Node::getEdgeName(p2, q2)] == 0) {
          next_node = u;
          break;
        }
      }

      // stay
      if (next_node == nullptr) next_node = v;
      C_next.push_back(next_node);
    }
    plan.add(C_next);
    C = C_next;
    C_next.clear();
  }
}
