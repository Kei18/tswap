#include "../include/pibt_complete.hpp"

#include <fstream>

#include "../include/icbs.hpp"
#include "../include/ecbs.hpp"
#include "../include/pibt.hpp"
#include "../include/goal_allocator.hpp"

const std::string PIBT_COMPLETE::SOLVER_NAME = "PIBT_COMPLETE";

PIBT_COMPLETE::PIBT_COMPLETE(Problem* _P)
  : Solver(_P),
    complement_solver_type(COMPLEMENT_SOLVER_TYPE::S_ECBS),
    ecbs_suboptimality(1.1)
{
  solver_name = SOLVER_NAME;
  comp_time_complement = 0;
}

void PIBT_COMPLETE::run()
{
  // goal assignment
  GoalAllocator allocator = GoalAllocator(P);
  allocator.assign();
  auto goals = allocator.getAssignedGoals();

  info(" ", "elapsed:", getSolverElapsedTime(), ", finish goal assignment");

  // find lower bound of makespan
  int LB_makespan = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    int d = pathDist(P->getStart(i), goals[i]);
    if (d > LB_makespan) LB_makespan = d;
  }

  // solve by PIBT
  Problem _P = Problem(P, P->getConfigStart(), goals, max_comp_time, LB_makespan);
  std::unique_ptr<Solver> init_solver = std::make_unique<PIBT>(&_P);
  info(" ", "run PIBT until timestep", LB_makespan);
  init_solver->solve();
  solution = init_solver->getSolution();

  if (init_solver->succeed()) {  // PIBT success
    solved = true;

  } else {  // PIBT failed

    auto t_complement = Time::now();

    // solved by ICBS
    int comp_time_limit = max_comp_time - (int)getSolverElapsedTime();
    Problem _Q = Problem(P, solution.last(), goals,
                         comp_time_limit, max_timestep - LB_makespan);

    // setup complement solver
    std::unique_ptr<Solver> second_solver;
    switch (complement_solver_type) {
    case COMPLEMENT_SOLVER_TYPE::S_ICBS:
      second_solver = std::make_unique<ICBS>(&_Q);
      break;
    case COMPLEMENT_SOLVER_TYPE::S_ECBS:
    default:
      second_solver = std::make_unique<ECBS>(&_Q);
      break;
    }

    // set ECBS suboptimality
    if (complement_solver_type == COMPLEMENT_SOLVER_TYPE::S_ECBS) {
      char arg0[] = "dummy";
      char arg1[] = "-w";
      auto str_w = std::to_string(ecbs_suboptimality);
      char arg2[str_w.size() + 1];
      for (int i = 0; i < str_w.size(); ++i) arg2[i] = str_w[i];
      char* argv[] = { arg0, arg1, arg2 };
      second_solver->setParams(3, argv);
    }

    info(" ", "elapsed:", getSolverElapsedTime(),
         ", use", second_solver->getSolverName(),
         "to complement the remain");

    second_solver->solve();
    solution += second_solver->getSolution();
    if (second_solver->succeed()) solved = true;

    comp_time_complement = getElapsedTime(t_complement);
  }
}

void PIBT_COMPLETE::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
    {"complement-solver", required_argument, 0, 'c'},
    {"ecbs-suboptimality", required_argument, 0, 'w'},
    {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "w:c:", longopts, &longindex)) != -1) {
    switch (opt) {
    case 'c':
      if (std::string(optarg) == "ICBS") {
        complement_solver_type = COMPLEMENT_SOLVER_TYPE::S_ICBS;
      } else if (std::string(optarg) == "ICBS") {
        complement_solver_type = COMPLEMENT_SOLVER_TYPE::S_ECBS;
      } else {
        warn("unknown type of complement solver");
      }
      break;
    case 'w':
      ecbs_suboptimality = std::atof(optarg);
      if (ecbs_suboptimality < 1) halt("sub-optimality should be >= 1");
      break;
    default:
      break;
    }
  }
}

void PIBT_COMPLETE::printHelp()
{
  std::cout << PIBT_COMPLETE::SOLVER_NAME << "\n"
            << "  -c --complement-solver"
            << "           "
            << "choose complement solver { ECBS, ICBS }\n"

            << "  -w --ecbs-suboptimality"
            << "           "
            << "set sub-optimality for ECBS, default 1.1"

            << std::endl;
}

void PIBT_COMPLETE::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);

  // print additional info
  log << "comp_time_complement=" << comp_time_complement << "\n";

  makeLogSolution(log);
  log.close();
}
