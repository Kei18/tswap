#include "../include/naive_tswap.hpp"

#include <fstream>

#include "../include/goal_allocator.hpp"

const std::string NaiveTSWAP::SOLVER_NAME = "NaiveTSWAP";

NaiveTSWAP::NaiveTSWAP(Problem* _P) : Solver(_P), use_bfs_allocate(false)
{
  solver_name = SOLVER_NAME;
}

NaiveTSWAP::~NaiveTSWAP() {}

void NaiveTSWAP::run()
{
  Plan plan;  // will be solution

  // goal assignment
  info(" ", "start task allocation");
  GoalAllocator allocator = GoalAllocator(P, use_bfs_allocate);
  allocator.assign();
  auto goals = allocator.getAssignedGoals();

  elapsed_assignment = getSolverElapsedTime();
  estimated_soc = allocator.getCost();
  estimated_makespan = allocator.getMakespan();

  info(" ", "elapsed:", elapsed_assignment, ", finish goal assignment",
       ", soc: >=", estimated_soc, ", makespan: >=", estimated_makespan);

  auto t_pathplanning = Time::now();

  // setup agent
  std::vector<Agent*> A;
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Node* g = goals[i];
    A.push_back(new Agent{i, s, g});
  }

  // set initial config
  plan.add(P->getConfigStart());

  // main loop
  int timestep = 0;
  while (true) {
    // planning
    for (auto a : A) {
      // already at goal
      if (a->v == a->g) continue;

      // desired node
      Node* u = getPath(a->v, a->g)[1];

      // simple implementation with O(A), for faster version, see tswap.cpp
      auto itr =
          std::find_if(A.begin(), A.end(), [u](Agent* b) { return u == b->v; });
      if (itr == A.end()) {
        a->v = u;
        continue;
      }

      Agent* b = *itr;
      if (b->v == b->g) {  // swap goal
        auto tmp = a->g;
        a->g = b->g;
        b->g = tmp;
      } else {                          // deadlock detection
        std::vector<Agent*> A_p = {a};  // A'
        while (true) {
          if (b->v == b->g) break;  // not deadlock
          Node* w = getPath(b->v, b->g)[1];
          auto itr_w = std::find_if(A.begin(), A.end(),
                                    [w](Agent* c) { return c->v == w; });
          if (itr_w == A.end()) break;  // not deadlock
          A_p.push_back(b);
          b = *itr_w;
          if (b == a) break;  // deadlock

          // there is a deadlock, but a is not in the deadlock
          if (inArray(b, A_p)) {
            A_p.clear();
            break;
          }
        }
        if (A_p.size() > 1 && b == a) {  // deadlock
          // rotate targets
          Node* g = (*(A_p.end() - 1))->g;
          for (auto itr = A_p.begin() + 1; itr != A_p.end(); ++itr)
            (*itr)->g = (*(itr - 1))->g;
          (*A_p.begin())->g = g;
        }
      }
    }

    // acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);
    for (int i = 0; i < P->getNum(); ++i) {
      auto a = A[i];
      // set next location
      config[i] = a->v;
      // check goal condition
      check_goal_cond &= (a->v == a->g);
    }

    // update plan
    plan.add(config);

    ++timestep;

    // success
    if (check_goal_cond) {
      solved = true;
      break;
    }

    // failed
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }
  }

  info(" ", "elapsed:", getSolverElapsedTime(), ", finish paht planning");

  elapsed_pathplanning = getElapsedTime(t_pathplanning);

  // free
  for (auto a : A) delete a;

  solution = plan;
}

void NaiveTSWAP::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"use-bfs-allocate", no_argument, 0, 'b'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "b", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'b':
        use_bfs_allocate = true;
        break;
      default:
        break;
    }
  }
}

void NaiveTSWAP::printHelp()
{
  std::cout << NaiveTSWAP::SOLVER_NAME << "\n"

            << "  -b --use-bfs-allocate"
            << "         "
            << "use BFS in goal allocation"

            << std::endl;
}

void NaiveTSWAP::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);

  log << "internal_info=\n"
      << "elapsed_assignment:" << elapsed_assignment << "\n"
      << "elapsed_path_planning:" << elapsed_pathplanning << "\n"
      << "estimated_soc:" << estimated_soc << "\n"
      << "estimated_makespan:" << estimated_makespan << "\n";

  makeLogSolution(log);
  log.close();
}
