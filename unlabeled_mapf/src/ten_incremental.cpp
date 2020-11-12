#include "../include/ten_incremental.hpp"

using NodeType = LibTEN::TEN_Node::NodeType;

TEN_INCREMENTAL::TEN_INCREMENTAL(Problem* const _P, const bool _filter, const bool _ilp)
  : TEN(_P, 0, _filter, _ilp), current_timestep(0)
{
}

TEN_INCREMENTAL::TEN_INCREMENTAL(Problem* const _P, const int _t, const bool _filter, const bool _ilp)
  : TEN(_P, _t-1, _filter, _ilp), current_timestep(_t-1)
{
  if (_t > 1) TEN::updateGraph();
}

TEN_INCREMENTAL::~TEN_INCREMENTAL() {}

void TEN_INCREMENTAL::update()
{
  ++current_timestep;
  TEN::update();
}

void TEN_INCREMENTAL::updateGraph()
{
  extendGraphOneTimestep(current_timestep);

  // add source
  if (current_timestep == 1) {
    for (auto v : P->getConfigStart()) {
      network.addParent(network.getNode(NodeType::V_IN, v, 1), network.source);
    }
  }

  // add sink
  for (auto v : P->getConfigGoal()) {
    network.addParent(network.sink, network.getNode(NodeType::V_OUT, v, current_timestep));
    network.sink->t = current_timestep;
  }

  // inherit flow of previous iteration
  if (current_timestep > 1) {
    for (auto v : P->getConfigGoal()) {
      auto p = network.getNode(NodeType::V_OUT, v, current_timestep - 1);
      // agent has already reached goal in previous iteration
      if (network.getCapacity(p, network.sink) == 0) {
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
