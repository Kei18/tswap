#include "../include/time_expanded_network.hpp"

#include <stack>
#include <string>
#include <unordered_map>

std::unordered_map<std::string, TEN_Node*> TEN_Node::all_nodes;

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
  for (auto itr = TEN_Node::all_nodes.begin(); itr != TEN_Node::all_nodes.end(); ++itr) {
    delete itr->second;
  }
}

void TimeExpandedNetwork::createGraph()
{
  // add vertex
  for (int t = 1; t <= T; ++t) {
    for (auto v : V) {
      auto v_in  = TEN_Node::createNewNode(TEN_Node::NodeType::V_IN,  v, t);
      auto v_out = TEN_Node::createNewNode(TEN_Node::NodeType::V_OUT, v, t);
      v_out->addParent(v_in);
      if (t > 1) {
        v_in->addParent(TEN_Node::getNode(TEN_Node::NodeType::V_OUT, v, t - 1));
      }
    }
  }

  // add edges
  for (int t = 1; t <= T; ++t) {
    for (auto v : V) {
      for (auto u : v->neighbor) {
        if (v->id >= u->id) continue;  // avoid duplication
        auto w_in  = TEN_Node::createNewNode(TEN_Node::NodeType::W_IN,  v, u, t);
        auto w_out = TEN_Node::createNewNode(TEN_Node::NodeType::W_OUT, v, u, t);
        w_in->addParent(TEN_Node::getNode(TEN_Node::NodeType::V_IN, v, t));
        w_in->addParent(TEN_Node::getNode(TEN_Node::NodeType::V_IN, u, t));
        w_out->addParent(w_in);
        TEN_Node::getNode(TEN_Node::NodeType::V_OUT, v, t)->addParent(w_out);
        TEN_Node::getNode(TEN_Node::NodeType::V_OUT, u, t)->addParent(w_out);
      }
    }
  }

  // add source
  source = TEN_Node::createNewNode(TEN_Node::NodeType::SOURCE);
  for (auto v : P->getConfigStart()) {
    TEN_Node::getNode(TEN_Node::NodeType::V_IN, v, 1)->addParent(source);
  }

  // add sink
  sink = TEN_Node::createNewNode(TEN_Node::NodeType::SINK);
  for (auto v : P->getConfigGoal()) {
    sink->addParent(TEN_Node::getNode(TEN_Node::NodeType::V_OUT, v, T));
  }
}

void TimeExpandedNetwork::FordFulkerson()
{
  // initialize
  for (auto itr = TEN_Node::all_nodes.begin(); itr != TEN_Node::all_nodes.end(); ++itr) {
    auto p = itr->second;
    for (auto q : p->children) {
      residual_capacity[TEN_Node::getEdgeName(p, q)] = 1;
      residual_capacity[TEN_Node::getEdgeName(q, p)] = 0;
    }
  }

  while (true) {
    // depth first search
    std::unordered_map<std::string, bool> CLOSED;
    auto dfs = [&](auto&& self, TEN_Node* p) -> TEN_Node* {
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
  solution.add(C);

  for (int t = 1; t <= T; ++t) {
    for (auto v : C) {
      Node* next_node = nullptr;
      // move
      for (auto u : v->neighbor) {
        auto first = (v->id < u->id) ? v : u;
        auto second = (v->id < u->id) ? u : v;
        auto p1 = TEN_Node::getNode(TEN_Node::NodeType::V_IN, v, t);
        auto q1 = TEN_Node::getNode(TEN_Node::NodeType::W_IN,  first, second, t);
        auto p2 = TEN_Node::getNode(TEN_Node::NodeType::W_OUT, first, second, t);
        auto q2 = TEN_Node::getNode(TEN_Node::NodeType::V_OUT, u, t);
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
    solution.add(C_next);
    C = C_next;
    C_next.clear();
  }
}
