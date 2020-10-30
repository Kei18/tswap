#include "../include/network_flow.hpp"

#include <memory>

#include "../include/ten.hpp"
#include "../include/ten_incremental.hpp"

const std::string NetworkFlow::SOLVER_NAME = "NetworkFlow";

NetworkFlow::NetworkFlow(Problem* _P)
  : Solver(_P),
    use_incremental(true),
    use_filter(true),
    use_minimum_step(false)
{
  solver_name = NetworkFlow::SOLVER_NAME;
}

NetworkFlow::~NetworkFlow() {}

void NetworkFlow::run()
{
  // determine lower bound
  int minimum_step = 1;
  if (use_minimum_step) {
    auto goals = P->getConfigGoal();
    for (auto s : P->getConfigStart()) {
      Node* g = *std::min_element(goals.begin(), goals.end(),
                                  [&](Node* v, Node* u)
                                  { return s->manhattanDist(v) < s->manhattanDist(u); });
      int d = s->manhattanDist(g);
      if (d > minimum_step) minimum_step = d;
    }
    info(" ", "elapsed: ", getSolverElapsedTime(), ", minimum_step:", minimum_step);
  }

  std::unique_ptr<TEN> flow_network;
  if (use_incremental) {
    flow_network = std::make_unique<TEN_INCREMENTAL>(P, minimum_step, use_filter);
  }

  for (int t = minimum_step; t <= max_timestep; ++t) {
    if (overCompTime()) break;
    if (!use_incremental) flow_network = std::make_unique<TEN>(P, t, use_filter);
    flow_network->update();

    float visited_rate = (float)flow_network->getDfsCnt() / flow_network->getNodesNum();
    info(" ", "elapsed:", getSolverElapsedTime(),
         ", makespan_limit:", t,
         ", visited_ndoes:", flow_network->getDfsCnt(),
         "/", flow_network->getNodesNum(), "=", visited_rate);

    if (flow_network->isValid()) {
      solved = true;
      solution = flow_network->getPlan();
      break;
    }
  }
}

void NetworkFlow::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
    {"no-cache", no_argument, 0, 'n'},
    {"no-filter", no_argument, 0, 'f'},
    {"use-minimum-step", no_argument, 0, 'm'},
    {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "nfm", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'n':
        use_incremental = false;
        break;
      case 'f':
        use_filter = false;
      case 'm':
        use_minimum_step = true;
      default:
        break;
    }
  }
}

void NetworkFlow::printHelp()
{
  std::cout << NetworkFlow::SOLVER_NAME << "\n"
            << "  -n --no-cache"
            << "                 "
            << "implement without cache\n"
            << "  -f --no-filter"
            << "                "
            << "implement without filter"
            << "  -m --use-minimum-step"
            << "          "
            << "implement with minimum-step"
            << std::endl;
}
