#include "../include/network_flow.hpp"

#include <memory>

#include "../include/ten.hpp"
#include "../include/ten_incremental.hpp"

const std::string NetworkFlow::SOLVER_NAME = "NetworkFlow";

NetworkFlow::NetworkFlow(Problem* _P)
    : Solver(_P),
      use_incremental(true),
      use_filter(true),
      use_minimum_step(false),
      use_ilp_solver(false)
{
  solver_name = NetworkFlow::SOLVER_NAME;
}

NetworkFlow::~NetworkFlow() {}

void NetworkFlow::run()
{
  // determine lower bouned with manhattanDist
  int minimum_step = 1;
  if (use_minimum_step) {
    auto goals = P->getConfigGoal();
    for (auto s : P->getConfigStart()) {
      Node* g =
          *std::min_element(goals.begin(), goals.end(), [&](Node* v, Node* u) {
            return s->manhattanDist(v) < s->manhattanDist(u);
          });
      int d = s->manhattanDist(g);
      if (d > minimum_step) minimum_step = d;
    }
    info(" ", "elapsed: ", getSolverElapsedTime(),
         ", minimum_step:", minimum_step);
  }

  std::unique_ptr<TEN> flow_network;
  if (use_incremental) {
    flow_network = std::make_unique<TEN_INCREMENTAL>(
        P, minimum_step, use_filter, use_ilp_solver);
  }

  for (int t = minimum_step; t <= max_timestep; ++t) {
    if (overCompTime()) break;
    if (!use_incremental)
      flow_network = std::make_unique<TEN>(P, t, use_filter, use_ilp_solver);

    // set time limit
    flow_network->setTimeLimit(max_comp_time - (int)getSolverElapsedTime());

    // update network
    flow_network->update();

    if (use_ilp_solver) {
#ifdef _GUROBI_
      info(" ", "elapsed:", getSolverElapsedTime(), ", makespan_limit:", t,
           ", variants:", flow_network->getVariantsCnt(),
           ", constraints:", flow_network->getConstraintsCnt());
#endif
    } else {
      float visited_rate =
          (float)flow_network->getDfsCnt() / flow_network->getNodesNum();
      info(" ", "elapsed:", getSolverElapsedTime(), ", makespan_limit:", t,
           ", visited_ndoes:", flow_network->getDfsCnt(), "/",
           flow_network->getNodesNum(), "=", visited_rate);
    }

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
      {"use-ilp-solver", no_argument, 0, 'g'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "nfmg", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'n':
        use_incremental = false;
        break;
      case 'f':
        use_filter = false;
        break;
      case 'm':
        use_minimum_step = true;
        break;
#ifdef _GUROBI_
      case 'g':
        use_ilp_solver = true;
        break;
#endif
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
            << "implement without filter\n"
            << "  -m --use-minimum-step"
            << "          "
            << "implement with minimum-step (Manhattan distance)";

#ifdef _GUROBI_
  std::cout << "\n  -g --use-ilp-solver"
            << "           "
            << "implement with ILP solver (GUROBI)";
#endif

  std::cout << std::endl;
}
