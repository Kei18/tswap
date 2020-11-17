#include "../include/ten.hpp"

using NodeType = LibTEN::TEN_Node::NodeType;

TEN::TEN(Problem* const _P, const int _T, const bool _filter,
         const bool _use_ilp_solver)
    : P(_P),
      V(P->getG()->getV()),
      network(LibTEN::ResidualNetwork(_filter, _use_ilp_solver, P)),
      valid_network(false),
      max_timestep(_T)
{
}

TEN::~TEN() {}

void TEN::update()
{
  updateGraph();
  network.solve();
  valid_network = (network.getFlowSum() == P->getNum());
  createPlan();
}

void TEN::extendGraphOneTimestep(const int t)
{
  for (auto v : V) {
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
      auto w_in = network.createNewNode(NodeType::W_IN, u, v, t);
      auto w_out = network.createNewNode(NodeType::W_OUT, u, v, t);
      network.addParent(w_in, network.getNode(NodeType::V_IN, u, t));
      network.addParent(w_in, network.getNode(NodeType::V_IN, v, t));
      network.addParent(w_out, w_in);
      network.addParent(network.getNode(NodeType::V_OUT, u, t), w_out);
      network.addParent(network.getNode(NodeType::V_OUT, v, t), w_out);
    }
  }
}

void TEN::updateGraph()
{
  for (int t = 1; t <= max_timestep; ++t) extendGraphOneTimestep(t);

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
        auto p1 = network.getNode(NodeType::V_IN, v, t);
        auto q1 = network.getNode(NodeType::W_IN, first, second, t);
        auto p2 = network.getNode(NodeType::W_OUT, first, second, t);
        auto q2 = network.getNode(NodeType::V_OUT, u, t);
        if (network.getCapacity(p1, q1) == 0 &&
            network.getCapacity(p2, q2) == 0) {
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

int TEN::getNodesNum() { return network.getNodesNum(); }

int TEN::getEdgesNum() { return network.getEdgesNum(); }

int TEN::getDfsCnt() { return network.dfs_cnt; };

#ifdef _GUROBI_
int TEN::getVariantsCnt() { return network.variants_cnt; };
int TEN::getConstraintsCnt() { return network.constraints_cnt; };
#endif
