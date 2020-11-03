#include "../include/ir.hpp"

#include <fstream>
#include <set>

const std::string IR::SOLVER_NAME = "IR";
const int IR::DEFAULT_MAX_ITERATION = 100;

IR::IR(Problem* _P) : Solver(_P)
{
  solver_name = IR::SOLVER_NAME;

  output_file = DEFAULT_OUTPUT_FILE;
  max_iteration = DEFAULT_MAX_ITERATION;
  verbose_underlying_solver = false;
  make_log_every_itr = false;
  timeout_refinement = max_comp_time;
  HIST_GROUP_SIZE.push_back(0);
  HIST_GAP.push_back(0);
}

IR::~IR() {}

void IR::run()
{
  // get initial plan
  solution = getInitialPlan();
  solved = !solution.empty();
  if (!solved) return;  // failure
  Plan init_plan = solution;
  int last_soc = init_plan.getSOC();
  HIST.push_back(std::make_tuple(getSolverElapsedTime(), solution));

  // start refinement
  while (true) {
    if (make_log_every_itr) makeLog(output_file);
    if (overCompTime()) break;

    // print info
    int soc = solution.getSOC();
    info("  iter: ", HIST.size(), ", comp_time:", getSolverElapsedTime(),
         ", soc:", soc, "(improved: ", last_soc - soc,
         ")"
         ", makespan:",
         solution.getMakespan());
    last_soc = solution.getSOC();

    // refine plan
    solution = refinePlan(P->getConfigStart(), P->getConfigGoal(), solution);
    HIST.push_back(std::make_tuple(getSolverElapsedTime(), solution));
    if (stopRefinement()) break;
  }

  // print final info
  info("  refinement results, soc:", init_plan.getSOC(), "->",
       solution.getSOC(), ", makespan:", init_plan.getMakespan(), "->",
       solution.getMakespan());
}

// failed -> return empty plan
Plan IR::getInitialPlan()
{
  // set problem
  Problem _P = Problem(P, P->getConfigStart(), P->getConfigGoal(),
                       max_comp_time, max_timestep);

  // set solver
  std::unique_ptr<Solver> solver = std::make_unique<PIBT_COMPLETE>(&_P);

  // set solver options
  solver->setVerbose(verbose_underlying_solver);

  // solve
  solver->solve();

  // success
  Plan plan;
  if (solver->succeed()) plan = solver->getSolution();

  return plan;
}

Plan IR::refinePlan(const Config& config_s, const Config& config_g,
                    const Plan& current_plan)
{
  Paths current_paths = planToPaths(current_plan);
  auto gap = [&](int i) { return current_paths.costOfPath(i) - pathDist(i); };

  // find agent with largest gap
  int id_largest_gap = -1;
  int gap_largest = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    if (inArray(i, CLOSE)) continue;
    int gap_i = gap(i);
    if (gap_i > gap_largest) {
      id_largest_gap = i;
      gap_largest = gap_i;
    }
  }

  // failed to find the agent with largest gap
  if (id_largest_gap == -1) {
    CLOSE.clear();
    HIST_GROUP_SIZE.push_back(0);
    HIST_GAP.push_back(0);
    return current_plan;
  }

  // find interacting agents
  std::vector<int> sample = getInteractingAgents(current_paths, id_largest_gap);
  HIST_GROUP_SIZE.push_back(sample.size());
  HIST_GAP.push_back(gap_largest);
  info("   ", "id=", id_largest_gap, ", gap=", gap_largest,
       ", interacting size:", sample.size());

  // create problem
  int comp_time_limit =
      std::min(max_comp_time - (int)getSolverElapsedTime(), timeout_refinement);
  if (comp_time_limit <= 0) return current_plan;  // timeout
  Problem _P = Problem(P, config_s, config_g, comp_time_limit, max_timestep);

  // solve
  auto res = getOptimalPlan(&_P, current_plan, sample);

  Plan plan = std::get<1>(res);
  if (!std::get<0>(res) || plan.getSOC() == current_plan.getSOC()) {
    CLOSE.push_back(id_largest_gap);
  }
  return plan;
}

std::vector<int> IR::getInteractingAgents(const Paths& current_paths,
                                          const int id_largest_gap)
{
  int cost_largest_gap = current_paths.costOfPath(id_largest_gap);
  int dist_largest_gap = pathDist(id_largest_gap);
  std::set<int> sample = {id_largest_gap};
  Node* g = P->getGoal(id_largest_gap);
  for (int t = cost_largest_gap - 1; t >= dist_largest_gap; --t) {
    for (int i = 0; i < P->getNum(); ++i) {
      if (i == id_largest_gap) continue;
      if (current_paths.get(i, t) == g) sample.insert(i);
    }
  }
  std::vector<int> sample_vec(sample.begin(), sample.end());
  return sample_vec;
}

std::tuple<bool, Plan> IR::getOptimalPlan(Problem* _P, const Plan& current_plan,
                                          const std::vector<int>& sample)
{
  // set solver
  std::unique_ptr<Solver> solver =
      std::make_unique<ICBS_REFINE>(_P, current_plan, sample);

  // set solver option
  solver->setVerbose(verbose_underlying_solver);

  // solve
  solver->solve();

  Plan plan = current_plan;
  bool success = false;

  // success
  if (solver->succeed()) {
    plan = solver->getSolution();
    success = true;
  }

  return std::make_tuple(success, plan);
}

bool IR::stopRefinement() { return HIST.size() >= max_iteration; }

void IR::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"timeout-refinement", required_argument, 0, 't'},
      {"log-every-iter", required_argument, 0, 'l'},
      {"verbose-underlying", no_argument, 0, 'V'},
      {"max-iteration", required_argument, 0, 'n'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  std::string s, s_tmp;

  while ((opt = getopt_long(argc, argv, "o:lt:x:y:X:Y:Vn:", longopts,
                            &longindex)) != -1) {
    switch (opt) {
      case 'o':
        output_file = std::string(optarg);
        break;
      case 'l':
        make_log_every_itr = true;
        break;
      case 't':
        timeout_refinement = std::atoi(optarg);
        if (timeout_refinement < 0 || max_comp_time < timeout_refinement) {
          warn("invalid early-stop time, using max_comp_time");
          timeout_refinement = max_comp_time;
        }
        break;
      case 'V':
        verbose_underlying_solver = true;
        break;
      case 'n':
        max_iteration = std::atoi(optarg);
        break;
      default:
        break;
    }
  }
}

void IR::printHelp()
{
  std::cout << IR::SOLVER_NAME << "\n"
            << "  -l --log-every-iter"
            << "           "
            << "make log for every iteration\n"

            << "  -t --timeout-refinement [INT]"
            << " "
            << "timeout for refinement\n"

            << "  -n --max-iteration [INT]"
            << "      "
            << "max iteration\n"

            << std::endl;
}

void IR::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);

  // record the data of each iteration
  for (int t = 0; t < HIST.size(); ++t) {
    Plan plan = std::get<1>(HIST[t]);
    log << "iter=" << t << ","
        << "comp_time=" << std::get<0>(HIST[t]) << ","
        << "soc=" << plan.getSOC() << ","
        << "makespan=" << plan.getMakespan() << ","
        << "group=" << HIST_GROUP_SIZE[t] << ","
        << "gap=" << HIST_GAP[t] << "\n";
  }

  makeLogSolution(log);
  log.close();
}
