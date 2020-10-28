#include "../include/network_flow.hpp"
#include "../include/ten.hpp"
#include "../include/ten_incremental.hpp"
#include <memory>

const std::string NetworkFlow::SOLVER_NAME = "NetworkFlow";

NetworkFlow::NetworkFlow(Problem* _P) : Solver(_P), use_incremental(true)
{
  solver_name = NetworkFlow::SOLVER_NAME;
}

NetworkFlow::~NetworkFlow() {}

void NetworkFlow::run()
{
  TEN* flow_network;
  if (use_incremental) flow_network = new TEN_INCREMENTAL(P);

  // simple test
  for (int t = 1; t <= max_timestep; ++t) {
    if (overCompTime()) break;

    info(" ", "elapsed:", getSolverElapsedTime(), ", makespan_limit:", t);

    if (!use_incremental) flow_network = new TEN(P, t);
    flow_network->update();

    if (flow_network->isValid()) {
      solved = true;
      solution = flow_network->getPlan();
      break;
    }

    if (!use_incremental) delete flow_network;
  }

  if (use_incremental) delete flow_network;
}

void NetworkFlow::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
    {"no-cache", no_argument, 0, 'n'},
    {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "n", longopts, &longindex)) != -1) {
    switch (opt) {
    case 'n':
      use_incremental = false;
      break;
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
            << "implement without cache" << std::endl;
}
