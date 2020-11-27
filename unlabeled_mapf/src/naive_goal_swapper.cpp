#include "../include/naive_goal_swapper.hpp"

#include "../include/goal_allocator.hpp"

const std::string NaiveGoalSwapper::SOLVER_NAME = "NaiveGoalSwapper";

NaiveGoalSwapper::NaiveGoalSwapper(Problem* _P)
  : Solver(_P),
    use_bfs_allocate(false)
{
  solver_name = SOLVER_NAME;
}

NaiveGoalSwapper::~NaiveGoalSwapper() {}

void NaiveGoalSwapper::run()
{
  Plan plan;  // will be solution

  // goal assignment
  info(" ", "start task allocation");
  GoalAllocator allocator = GoalAllocator(P, use_bfs_allocate);
  allocator.assign();
  auto goals = allocator.getAssignedGoals();
  info(" ", "elapsed:", getSolverElapsedTime(),
       ", finish goal assignment",
       ", soc: >=", allocator.getCost(),
       ", makespan: >=", allocator.getMakespan());

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

      // simple implementation with O(A), for faster version, see goal_swapper
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

  // free
  for (auto a : A) delete a;

  solution = plan;
}

void NaiveGoalSwapper::setParams(int argc, char* argv[])
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

void NaiveGoalSwapper::printHelp()
{
  std::cout << NaiveGoalSwapper::SOLVER_NAME << "\n"

            << "  -b --use-bfs-allocate"
            << "         "
            << "use BFS in goal allocation"

            << std::endl;
}
