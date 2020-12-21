#include "../include/ten_incremental.hpp"

using NodeType = LibTEN::TEN_Node::NodeType;

TEN_INCREMENTAL::TEN_INCREMENTAL(Problem* const _P,
                                 const bool _filter,
                                 const bool _ilp)
  : TEN(_P, 0, _filter, _ilp),
    current_timestep(0)
{
}

TEN_INCREMENTAL::TEN_INCREMENTAL(Problem* const _P,
                                 const int _t,
                                 const bool _filter,
                                 const bool _ilp,
                                 const int _time_limit)
  : TEN(_P, _t - 1, _filter, _ilp),
    current_timestep(_t - 1)
{
  setTimeLimit(_time_limit);
  if (_t > 1) TEN::updateGraph();
}

TEN_INCREMENTAL::~TEN_INCREMENTAL() {}

void TEN_INCREMENTAL::update()
{
  ++current_timestep;
  TEN::update();
}

void TEN_INCREMENTAL::update(const int t)
{
  // used in binary search, shrink the network
  if (current_timestep > t) {
    // clear capacity
    network.clearAllCapacity();

    // update sink
    network.sink->t = t;
    auto parents = network.sink->parents;
    for (auto q : parents) network.removeParent(network.sink, q);

    // delete edge t -> t+1, add sink edge
    for (auto v : V) {
      auto p = network.getNode(NodeType::V_OUT, v, t);
      auto q = network.getNode(NodeType::V_IN, v, t+1);
      network.removeParent(q, p);
      if (inArray(v, P->getConfigGoal())) network.addParent(network.sink, p);
    }

    current_timestep = t;

    // used in binary search
    // extend network
  } else if (network.getNode(NodeType::V_OUT, V[0], t) != nullptr) {
    // already created -> reuse
    for (auto v : V) {
      // update connectivity
      auto p = network.getNode(NodeType::V_OUT, v, current_timestep);
      auto q = network.getNode(NodeType::V_IN, v, current_timestep + 1);
      network.addParent(q, p);

      // remove redundant edge
      auto r = network.getNode(NodeType::V_IN, v, t + 1);
      if (r != nullptr) network.removeParent(r, network.getNode( NodeType::V_OUT, v, t));

      // update flow
      if (inArray(network.sink, p->children)) {
        // connect to sink
        network.addParent(network.sink, network.getNode(NodeType::V_OUT, v, t));
        // check flow
        if (network.getCapacity(p, network.sink) == 0) {
          for (int _t = current_timestep + 1; _t <= t; ++_t) {
            auto a = network.getNode(NodeType::V_OUT, v, _t - 1);
            auto b = network.getNode(NodeType::V_IN, v, _t);
            auto c = network.getNode(NodeType::V_OUT, v, _t);
            network.setFlow(a, b);
            network.setFlow(b, c);
            if (_t == t) network.setFlow(c, network.sink);
          }
        }
        network.removeParent(network.sink, p);
        network.deleteEdge(p, network.sink);
      }
    }

    network.sink->t = t;
    current_timestep = t;

  } else {
    // used in both incremental and binary search, extend the network
    while (current_timestep < t) {
      ++current_timestep;
      updateGraph();
      if (overCompTime()) return;
    }
  }

  // check time limit
  if (overCompTime()) return;

  network.solve();
  valid_network = (network.getFlowSum() == P->getNum());
  createPlan();
}

void TEN_INCREMENTAL::updateGraph()
{
  extendGraphOneTimestep(current_timestep);

  if (overCompTime()) return;

  // add source
  if (current_timestep == 1) {
    for (auto v : P->getConfigStart()) {
      network.addParent(network.getNode(NodeType::V_IN, v, 1), network.source);
    }
  }

  // add sink
  for (auto v : P->getConfigGoal()) {
    network.addParent(network.sink,
                      network.getNode(NodeType::V_OUT, v, current_timestep));
    network.sink->t = current_timestep;
  }

  if (overCompTime()) return;

  // inherit flow of previous iteration
  if (current_timestep > 1) {
    for (auto v : P->getConfigGoal()) {
      auto p = network.getNode(NodeType::V_OUT, v, current_timestep - 1);
      // agent has already reached goal in previous iteration
      if (!network.use_ilp_solver &&
          network.getCapacity(p, network.sink) == 0) {
        auto p = network.getNode(NodeType::V_OUT, v, current_timestep - 1);
        auto q = network.getNode(NodeType::V_IN, v, current_timestep);
        auto r = network.getNode(NodeType::V_OUT, v, current_timestep);
        network.setFlow(p, q);
        network.setFlow(q, r);
        network.setFlow(r, network.sink);
      }
      network.deleteEdge(p, network.sink);
      network.removeParent(network.sink, p);
    }
  }
}

void TEN_INCREMENTAL::createPlan() { TEN::createPlan(current_timestep); }
