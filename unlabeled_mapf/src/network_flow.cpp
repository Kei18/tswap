#include "../include/network_flow.hpp"

#include <memory>

#include "../include/ten.hpp"
#include "../include/ten_incremental.hpp"

const std::string NetworkFlow::SOLVER_NAME = "NetworkFlow";

NetworkFlow::NetworkFlow(Problem* _P)
  : Solver(_P),
    use_incremental(true),
    use_filter(true)
{
  solver_name = NetworkFlow::SOLVER_NAME;
}

NetworkFlow::~NetworkFlow() {}

void NetworkFlow::run()
{
  std::unique_ptr<TEN> flow_network;
  if (use_incremental) {
    flow_network = std::make_unique<TEN_INCREMENTAL>(P, use_filter);
  }

  for (int t = 1; t <= max_timestep; ++t) {
    if (overCompTime()) break;

    info(" ", "elapsed:", getSolverElapsedTime(), ", makespan_limit:", t);

    if (!use_incremental) {
      flow_network = std::make_unique<TEN>(P, t, use_filter);
    }
    flow_network->update();

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
    {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "nf", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'n':
        use_incremental = false;
        break;
      case 'f':
        use_filter = false;
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
            << std::endl;
}
