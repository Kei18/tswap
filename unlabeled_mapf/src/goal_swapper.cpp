#include "../include/goal_swapper.hpp"

#include "../include/goal_allocator.hpp"

const std::string GoalSwapper::SOLVER_NAME = "GoalSwapper";

GoalSwapper::GoalSwapper(Problem* _P) : Solver(_P)
{
  solver_name = SOLVER_NAME;
}

GoalSwapper::~GoalSwapper() {}

void GoalSwapper::run()
{
  Plan plan;  // will be solution

  // goal assignment
  info(" ", "start task allocation");
  GoalAllocator allocator = GoalAllocator(P);
  allocator.assign();
  auto goals = allocator.getAssignedGoals();

  info(" ", "elapsed:", getSolverElapsedTime(), ", finish goal assignment");

  // compare priority of agents
  auto compare = [](Agent* a, const Agent* b) {
    if (a->called != b->called) return a->called > b->called;
    if (a->v_now != a->g) return false;
    if (b->v_now != b->g) return true;
    return a < b;
  };

  // agents have not decided their next locations
  std::priority_queue<Agent*, std::vector<Agent*>, decltype(compare)>
    undecided(compare);

  // work as reservation table
  std::unordered_map<Node*, Agent*> occupied_now;
  std::unordered_map<Node*, Agent*> occupied_next;

  // move action
  auto moveTo = [&](Agent* a, Node* v) {
    a->v_next = v;
    occupied_next[v] = a;
  };
  auto stay = [&](Agent* a) { moveTo(a, a->v_now); };

  // change goal
  auto swapGoal = [&](Agent* a, Agent* b) {
    Node* v = b->g;
    b->g = a->g;
    a->g = v;
  };

  // all agents
  Agent A[P->getNum()];

  // setup agents
  for (int i = 0; i < P->getNum(); ++i) {
    auto a = &(A[i]);
    a->id = i;  // id
    a->v_now = P->getStart(i);  // current node
    a->v_next = nullptr;  // next node
    a->g = goals[i];  // goal
    a->called = 0;  // how many times an agent is called
    occupied_now[a->v_now] = a;

    // insert OPEN set
    undecided.push(a);
  }

  // set initial config
  plan.add(P->getConfigStart());

  // main loop
  int timestep = 0;
  while (true) {
    info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

    // planning
    while (!undecided.empty()) {
      // pickup one agent
      Agent* a_i = undecided.top();
      undecided.pop();
      a_i->called++;

      // rule 1. stay goal
      if (a_i->v_now == a_i->g) {
        stay(a_i);
        continue;
      }

      // desired node
      Node* u = planOneStep(a_i, occupied_now, occupied_next);

      // rule 2. if u is occupied in next timestep -> stay
      auto itr_next = occupied_next.find(u);
      if (itr_next != occupied_next.end()) {
        Agent* a_j = itr_next->second;
        if (a_j->v_next == a_j->g) swapGoal(a_i, a_j);
        stay(a_i);
        continue;
      }

      // rule 3. if u is occupied in current timestep
      auto itr_now = occupied_now.find(u);
      if (itr_now != occupied_now.end()) {
        Agent* a_j = itr_now->second;
        if (a_j->v_now == a_j->g) swapGoal(a_i, a_j);
        if (a_j->v_next == nullptr) {
          // skip
          undecided.push(a_i);
        } else if (a_j->v_next == u) {
          // wait
          stay(a_i);
        } else {
          // move
          moveTo(a_i, u);
        }
        continue;
      }

      // rule4. otherwise (u is free)
      moveTo(a_i, u);
      continue;
    }

    // clear
    occupied_now.clear();
    occupied_next.clear();

    // acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);
    for (int i = 0; i < P->getNum(); ++i) {
      Agent* a = &(A[i]);
      // set next location
      config[i] = a->v_next;
      occupied_now[a->v_next] = a;
      // check goal condition
      check_goal_cond &= (a->v_next == a->g);
      // reset params
      a->v_now = a->v_next;
      a->v_next = nullptr;
      a->called = 0;
      // push to priority queue
      undecided.push(a);
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

  solution = plan;
}

Node* GoalSwapper::planOneStep(Agent* a,
                               std::unordered_map<Node*, Agent*>& occupied_now,
                               std::unordered_map<Node*, Agent*>& occupied_next)
{
  Nodes C = a->v_now->neighbor;

  // goal exists -> return immediately
  if (inArray(a->g, C)) return a->g;

  // randomize
  std::shuffle(C.begin(), C.end(), *MT);

  return *std::min_element(C.begin(), C.end(), [&](Node* v, Node* u) {
    // path distance
    int c_v = pathDist(v, a->g);
    int c_u = pathDist(u, a->g);
    if (c_v != c_u) return c_v < c_u;
    // occupancy for next timestep
    int o_v_next = (int)(occupied_next.find(v) != occupied_next.end());
    int o_u_next = (int)(occupied_next.find(u) != occupied_next.end());
    if (o_v_next != o_u_next) return o_v_next < o_u_next;
    // occupancy for current timestep
    int o_v_now = (int)(occupied_now.find(v) != occupied_now.end());
    int o_u_now = (int)(occupied_now.find(u) != occupied_now.end());
    if (o_v_now != o_u_now) return o_v_now < o_u_now;
    return getRandomBoolean(MT);
  });
}

void GoalSwapper::printHelp()
{
  std::cout << GoalSwapper::SOLVER_NAME << "\n"
            << "  (no option)" << std::endl;
}
