#include "../include/incremental_time_expanded_network.hpp"

#include <string>
#include <unordered_map>

IncrementalTimeExpandedNetwork::IncrementalTimeExpandedNetwork(Problem* _P)
  : P(_P), V(P->getG()->getV()), T(0)
{
  source = TEN_Node::createNewNode(TEN_Node::NodeType::SOURCE);
  sink = TEN_Node::createNewNode(TEN_Node::NodeType::SINK);
}

IncrementalTimeExpandedNetwork::~IncrementalTimeExpandedNetwork()
{
  // free
  for (auto itr = TEN_Node::all_nodes.begin(); itr != TEN_Node::all_nodes.end(); ++itr) {
    delete itr->second;
  }
  residual_capacity.clear();
}

void IncrementalTimeExpandedNetwork::update()
{
  ++T;
  updateGraph();
  FordFulkerson();
  checkFlow();
  createPlan();
}

void IncrementalTimeExpandedNetwork::checkFlow()
{
  // check validity
  int flow = 0;
  for (auto p : source->children) {
    flow += getResidualCapacity(p, source);
  }
  valid_network = (flow == P->getNum());
}

void IncrementalTimeExpandedNetwork::updateGraph()
{
  // affect from previous iteration
  Nodes U;

  // delete connection to sink
  if (T > 1) {
    for (auto v : P->getConfigGoal()) {
      auto p = TEN_Node::getNode(TEN_Node::NodeType::V_OUT, v, T-1);
      // agent has already reached goal in previous iteration
      if (getResidualCapacity(p, sink) == 0) U.push_back(v);
      sink->removeParent(p);
    }
  }

  for (auto v : V) {
    // add vertex
    auto v_in  = TEN_Node::createNewNode(TEN_Node::NodeType::V_IN,  v, T);
    auto v_out = TEN_Node::createNewNode(TEN_Node::NodeType::V_OUT, v, T);
    v_out->addParent(v_in);

    if (T > 1) {
      v_in->addParent(TEN_Node::getNode(TEN_Node::NodeType::V_OUT, v, T - 1));
    }

    // add edges
    for (auto u : v->neighbor) {
      if (u->id >= v->id) continue;  // avoid duplication, already created?
      // u->id < v->id
      auto w_in  = TEN_Node::createNewNode(TEN_Node::NodeType::W_IN,  u, v, T);
      auto w_out = TEN_Node::createNewNode(TEN_Node::NodeType::W_OUT, u, v, T);
      w_in->addParent(TEN_Node::getNode(TEN_Node::NodeType::V_IN, u, T));
      w_in->addParent(TEN_Node::getNode(TEN_Node::NodeType::V_IN, v, T));
      w_out->addParent(w_in);
      TEN_Node::getNode(TEN_Node::NodeType::V_OUT, u, T)->addParent(w_out);
      TEN_Node::getNode(TEN_Node::NodeType::V_OUT, v, T)->addParent(w_out);
    }
  }

  // add source
  if (T == 1) {
    for (auto v : P->getConfigStart()) {
      TEN_Node::getNode(TEN_Node::NodeType::V_IN, v, 1)->addParent(source);
    }
  }

  // add sink
  for (auto v : P->getConfigGoal()) {
    sink->addParent(TEN_Node::getNode(TEN_Node::NodeType::V_OUT, v, T));
  }

  // effect from previous iteration
  for (auto v : U) {
    auto p = TEN_Node::getNode(TEN_Node::NodeType::V_OUT, v, T-1);
    auto q = TEN_Node::getNode(TEN_Node::NodeType::V_IN,  v, T);
    auto r = TEN_Node::getNode(TEN_Node::NodeType::V_OUT, v, T);
    decrementResidualCapacity(p, q);
    incrementResidualCapacity(q, p);
    decrementResidualCapacity(q, r);
    incrementResidualCapacity(r, q);
    decrementResidualCapacity(r, sink);
    incrementResidualCapacity(sink, r);
    residual_capacity.erase(TEN_Node::getEdgeName(p, sink));
    residual_capacity.erase(TEN_Node::getEdgeName(sink, p));
  }
}

void IncrementalTimeExpandedNetwork::FordFulkerson()
{
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

        int cap = getResidualCapacity(p, q);
        if (cap == 0) continue;
        auto res = self(self, q);
        if (res != nullptr) {
          decrementResidualCapacity(p, q);
          incrementResidualCapacity(q, p);
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
    flow += getResidualCapacity(p, source);
  }
  valid_network = (flow == P->getNum());
}

void IncrementalTimeExpandedNetwork::createPlan()
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
        if (getResidualCapacity(p1, q1) == 0 && getResidualCapacity(p2, q2) == 0) {
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

int IncrementalTimeExpandedNetwork::getResidualCapacity(TEN_Node* p, TEN_Node* q)
{
  std::string key = TEN_Node::getEdgeName(p, q);
  auto itr = residual_capacity.find(key);
  if (itr != residual_capacity.end()) return itr->second;

  // require initialization
  initResidualCapacity(p, q);
  return residual_capacity[key];
}

void IncrementalTimeExpandedNetwork::initResidualCapacity(TEN_Node* p, TEN_Node* q)
{
  std::string key1 = TEN_Node::getEdgeName(p, q);
  std::string key2 = TEN_Node::getEdgeName(q, p);
  if (inArray(q, p->children)) {
    residual_capacity[key1] = 1;
    residual_capacity[key2] = 0;
  } else if (inArray(q, p->parents)) {
    residual_capacity[key1] = 0;
    residual_capacity[key2] = 1;
  } else {
    halt("invalid residual capacity: " + p->name + " -> " + q->name);
  }
}

void IncrementalTimeExpandedNetwork::incrementResidualCapacity(TEN_Node* p, TEN_Node* q)
{
  int cap = getResidualCapacity(p, q);  // capacity
  residual_capacity[TEN_Node::getEdgeName(p, q)] = cap + 1;
}

void IncrementalTimeExpandedNetwork::decrementResidualCapacity(TEN_Node* p, TEN_Node* q)
{
  int cap = getResidualCapacity(p, q);  // capacity
  residual_capacity[TEN_Node::getEdgeName(p, q)] = cap - 1;
}


int IncrementalTimeExpandedNetwork::nodesNum()
{
  return TEN_Node::all_nodes.size();
}

int IncrementalTimeExpandedNetwork::edgesNum()
{
  int sum = 0;
  for (auto itr = TEN_Node::all_nodes.begin(); itr != TEN_Node::all_nodes.end(); ++itr) {
    sum += itr->second->children.size();
  }
  return sum;
}
