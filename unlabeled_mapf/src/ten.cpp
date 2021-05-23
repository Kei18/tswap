#include "../include/ten.hpp"

using NodeType = LibTEN::TEN_Node::NodeType;

TEN::TEN(Problem* const _P, const int _T, const bool _filter)
    : P(_P),
      V(P->getG()->getV()),
      network(LibTEN::ResidualNetwork(_filter, _P)),
      valid_network(false),
      time_limit(-1),
      max_timestep(_T)
{
}

TEN::~TEN() {}

void TEN::update()
{
  updateGraph();
  if (overCompTime()) return;  // check time limit
  network.solve();
  valid_network = (network.getFlowSum() == P->getNum());
  createPlan();
}

void TEN::update(const int t) { update(); }

void TEN::extendGraphOneTimestep(const int t)
{
  for (auto v : V) {
    if (overCompTime()) break;

    // add vertex
    auto v_in = network.createNewNode(NodeType::V_IN, v, t);
    auto v_out = network.createNewNode(NodeType::V_OUT, v, t);
    network.addParent(v_out, v_in);

    if (t > 1)
      network.addParent(v_in, network.getNode(NodeType::V_OUT, v, t - 1));

    // add edges
    for (auto u : v->neighbor) {
      if (u->id >= v->id) continue;  // avoid duplication
      // u->id < v->id
      network.addParent(network.getNode(NodeType::V_OUT, u, t), v_in);
      network.addParent(v_out, network.getNode(NodeType::V_IN, u, t));
    }
  }
}

void TEN::updateGraph()
{
  // construct body
  for (int t = 1; t <= max_timestep; ++t) {
    extendGraphOneTimestep(t);
    if (overCompTime()) return;
  }

  // add source
  for (auto v : P->getConfigStart()) {
    network.addParent(network.getNode(NodeType::V_IN, v, 1), network.source);
  }

  // add sink
  for (auto v : P->getConfigGoal()) {
    network.addParent(network.sink,
                      network.getNode(NodeType::V_OUT, v, max_timestep));
  }
  network.sink->t = max_timestep;
}

void TEN::createPlan() { createPlan(max_timestep); }

void TEN::createPlan(const int T)
{
  if (!valid_network) return;

  solution.clear();
  Config C = P->getConfigStart();
  Config C_next;
  solution.add(C);

  for (int t = 1; t <= T; ++t) {
    for (auto v : C) {
      Node* next_node = nullptr;
      // move action
      for (auto u : v->neighbor) {
        auto v_in = network.getNode(NodeType::V_IN, v, t);
        auto u_out = network.getNode(NodeType::V_OUT, u, t);
        // check intersection
        if (network.getCapacity(v_in, u_out) == 0) {
          auto u_in = network.getNode(NodeType::V_IN, u, t);
          auto v_out = network.getNode(NodeType::V_OUT, v, t);
          if (network.getCapacity(u_in, v_out) == 0) {
            next_node = v;  // stay
          } else {
            next_node = u;  // move
          }
          break;
        }
      }

      // stay action
      if (next_node == nullptr) next_node = v;
      C_next.push_back(next_node);
    }
    solution.add(C_next);
    C = C_next;
    C_next.clear();
  }
}

int TEN::getNodesNum() { return network.getNodesNum(); }

int TEN::getEdgesNum() { return network.getEdgesNum(); }

int TEN::getDfsCnt() { return network.dfs_cnt; }

void TEN::setTimeLimit(int _time_limit)
{
  time_limit = _time_limit;
  t_start = Time::now();
  network.setTimeLimit(_time_limit);
}

bool TEN::overCompTime() const
{
  if (time_limit == -1) return false;
  return getElapsedTime(t_start) >= time_limit;
}
