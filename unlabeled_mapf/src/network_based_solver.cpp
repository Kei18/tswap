#include "../include/network_based_solver.hpp"

#include "../include/time_expanded_network.hpp"

const std::string NetworkBasedSolver::SOLVER_NAME = "NetworkFlow";

NetworkBasedSolver::NetworkBasedSolver(Problem* _P) : Solver(_P)
{
  solver_name = NetworkBasedSolver::SOLVER_NAME;
}

NetworkBasedSolver::~NetworkBasedSolver() {}

void NetworkBasedSolver::run()
{
  // simple test
  for (int t = 1; t <= max_timestep; ++t) {
    if (overCompTime()) break;

    info(" ", "elapsed:", getSolverElapsedTime(), ", max_t:", t);

    TimeExpandedNetwork network = TimeExpandedNetwork(P, t);
    if (network.isValid()) {
      solved = true;
      solution = network.getPlan();
      break;
    }
  }
}

void NetworkBasedSolver::printHelp()
{
  std::cout << NetworkBasedSolver::SOLVER_NAME << "\n"
            << "  (no option)" << std::endl;
}
