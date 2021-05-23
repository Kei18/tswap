#include "../include/flow_network.hpp"

#include <fstream>
#include "../include/goal_allocator.hpp"

const std::string FlowNetwork::SOLVER_NAME = "FlowNetwork";

FlowNetwork::FlowNetwork(Problem* _P)
    : Solver(_P),
      use_aggressive_lower_bound(false),
      use_passive_lower_bound(false),
      use_binary_search(false),
      use_pruning(true),
      use_past_flow(true),
      use_incremental(true),
      minimum_step(1),
      is_optimal(false)
{
  solver_name = FlowNetwork::SOLVER_NAME;
}

FlowNetwork::~FlowNetwork() {}

void FlowNetwork::run()
{
  // setup minimum timestep
  if (use_aggressive_lower_bound) {
    // lazy evaluation, without min cost maximum matching
    GoalAllocator allocator = GoalAllocator(P, false, false);
    allocator.assign();
    minimum_step = allocator.getMakespan();
  } else if (use_passive_lower_bound) {
    auto goals = P->getConfigGoal();
    for (auto s : P->getConfigStart()) {
      Node* g =
          *std::min_element(goals.begin(), goals.end(), [&](Node* v, Node* u) {
            return s->manhattanDist(v) < s->manhattanDist(u);
          });
      int d = s->manhattanDist(g);
      if (d > minimum_step) minimum_step = d;
    }
  }

  info(" ", "elapsed: ", getSolverElapsedTime(),
       ", minimum_step:", minimum_step);

  std::shared_ptr<TEN> network_flow;
  if (use_incremental) {
    network_flow = std::make_shared<TEN_INCREMENTAL>(
        P, minimum_step, use_pruning,
        max_comp_time - (int)getSolverElapsedTime());
  }

  // for binary search
  int lower_bound = 0;
  int upper_bound = -1;
  int t_binary = 1;

  int t_real = minimum_step;
  while (t_real <= max_timestep && !overCompTime()) {
    // build time expanded network
    if (!use_incremental) {
      network_flow =
          std::make_shared<TEN>(P, t_real, use_pruning);
    } else if (!use_past_flow) {
      network_flow->resetFlow();
    }

    // set time limit
    network_flow->setTimeLimit(max_comp_time - (int)getSolverElapsedTime());

    // update network
    network_flow->update(t_real);

    // updte log
    HISTS.push_back({(int)getSolverElapsedTime(), t_real,
        network_flow->isValid(), network_flow->getDfsCnt(),
        network_flow->getNodesNum(), 0, 0});
    float visited_rate =
      (float)network_flow->getDfsCnt() / network_flow->getNodesNum();
    info(" ", "elapsed:", getSolverElapsedTime(), ", makespan_limit:", t_real,
         ", valid:", network_flow->isValid(),
         ", visited_nodes:", network_flow->getDfsCnt(), "/",
         network_flow->getNodesNum(), "=", visited_rate);

    // check solution
    if (network_flow->isValid()) {
      solved = true;
      solution = network_flow->getPlan();
      if (!use_binary_search) {
        is_optimal = true;
        break;
      }
      upper_bound = t_binary;
    } else {
      lower_bound = t_binary;
    }

    // update timestep
    if (!use_binary_search) {
      ++t_real;
    } else {
      t_binary = (upper_bound == -1)
                     ? t_binary * 2
                     : (upper_bound - lower_bound) / 2 + lower_bound;
      if (t_binary == lower_bound) {
        is_optimal = true;
        break;
      }
      t_real = t_binary + minimum_step - 1;  // for binary search
    }
  }
}

void FlowNetwork::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"no-cache", no_argument, 0, 'n'},
      {"use-aggressive-lower-bound", no_argument, 0, 'l'},
      {"use-passive-lower-bound", no_argument, 0, 'd'},
      {"use-binary-search", no_argument, 0, 'b'},
      {"no-pruning", no_argument, 0, 'p'},
      {"no-past-flow", no_argument, 0, 'r'},  // [r]euse
      {"use-ilp-solver", no_argument, 0, 'g'},
      {"start-timestep", no_argument, 0, 't'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "nlbprdgt:", longopts, &longindex)) !=
         -1) {
    switch (opt) {
      case 'n':
        use_incremental = false;
        break;
      case 'l':
        use_aggressive_lower_bound = true;
        break;
      case 'd':
        use_passive_lower_bound = true;
        break;
      case 'b':
        use_binary_search = true;
        break;
      case 'p':
        use_pruning = false;
        break;
      case 'r':
        use_past_flow = false;
        break;
      case 't':
        minimum_step = std::atoi(optarg);
        if (minimum_step <= 0) {
          minimum_step = 1;
          warn("start timestep should be greater than 0");
        }
        break;
      default:
        break;
    }
  }
}

void FlowNetwork::printHelp()
{
  std::cout << FlowNetwork::SOLVER_NAME << "\n"
            << "  -n --no-cache"
            << "                 "
            << "implement without cache\n"

            << "  -l --use-aggressive-lower-bound"
            << "          "
            << "LB, calculated by bottleneck assignment\n"

            << "  -d --use-passive-lower-bound"
            << "          "
            << "LB, calculated by Manhattan distance\n"

            << "  -b --use-binary-search"
            << "        "
            << "Binary, implement binary search for optimal makespan\n"

            << "  -p --no-pruning"
            << "               "
            << "no Pruning\n"

            << "  -r --no-past-flow"
            << "             "
            << "no Reuse, implement without past flow\n"

            << "  -t --start-timestep [INT]"
            << "     "
            << "start timestep";

  std::cout << std::endl;
}

void FlowNetwork::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);

  log << "params="
      << "\nuse_incremental:" << use_incremental
      << "\nuse_aggressive_lower_bound:" << use_aggressive_lower_bound
      << "\nuse_passive_lower_bound:" << use_passive_lower_bound
      << "\nuse_binary_search:" << use_binary_search
      << "\nuse_pruning:" << use_pruning
      << "\nuse_past_flow:" << use_past_flow
      << "\nminimum_step:" << minimum_step << "\n";
  log << "optimal=" << is_optimal << "\n";
  log << "history=\n";
  for (auto hist : HISTS) {
    log << "elapsed:" << hist.elapsed << ",makespan:" << hist.makespan
        << ",valid:" << hist.valid << ",network_size:" << hist.network_size
        << ",visited:" << hist.visited_nodes;
    log << "\n";
  }

  makeLogSolution(log);
  log.close();
}
