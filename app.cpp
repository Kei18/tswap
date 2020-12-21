#include <getopt.h>
#include <default_params.hpp>
#include <tswap.hpp>
#include <iostream>
#include <naive_tswap.hpp>
#include <network_flow.hpp>
#include <problem.hpp>
#include <random>
#include <util.hpp>
#include <vector>

void printHelp();
std::unique_ptr<Solver> getSolver(const std::string solver_name, Problem *P,
                                  bool verbose, int argc, char *argv[]);

int main(int argc, char *argv[])
{
  std::string instance_file = "";
  std::string output_file = DEFAULT_OUTPUT_FILE;
  std::string solver_name;
  bool verbose = false;
  char *argv_copy[argc + 1];
  for (int i = 0; i < argc; ++i) argv_copy[i] = argv[i];

  struct option longopts[] = {
      {"instance", required_argument, 0, 'i'},
      {"output", required_argument, 0, 'o'},
      {"solver", required_argument, 0, 's'},
      {"verbose", no_argument, 0, 'v'},
      {"help", no_argument, 0, 'h'},
      {"make-scen", no_argument, 0, 'P'},
      {0, 0, 0, 0},
  };
  bool make_scen = false;

  // command line args
  int opt, longindex;
  opterr = 0;  // ignore getopt error
  while ((opt = getopt_long(argc, argv, "i:o:s:vhP", longopts, &longindex)) !=
         -1) {
    switch (opt) {
      case 'i':
        instance_file = std::string(optarg);
        break;
      case 'o':
        output_file = std::string(optarg);
        break;
      case 's':
        solver_name = std::string(optarg);
        break;
      case 'v':
        verbose = true;
        break;
      case 'h':
        printHelp();
        return 0;
      case 'P':
        make_scen = true;
        break;
      default:
        break;
    }
  }

  if (instance_file.length() == 0) {
    std::cout << "specify instance file using -i [INSTANCE-FILE], e.g.,"
              << std::endl;
    std::cout << "> ./app -i ../instance/sample.txt" << std::endl;
    return 0;
  }

  // set problem
  Problem P = Problem(instance_file);

  // create scenario
  if (make_scen) {
    P.makeScenFile(output_file);
    return 0;
  }

  // solve
  std::unique_ptr<Solver> solver =
      getSolver(solver_name, &P, verbose, argc, argv_copy);
  solver->solve();
  if (solver->succeed() && !solver->getSolution().validate(&P)) {
    halt("invalid results");
  }
  solver->printResult();

  // output result
  solver->makeLog(output_file);
  if (verbose) {
    std::cout << "save result as " << output_file << std::endl;
  }

  return 0;
}

std::unique_ptr<Solver> getSolver(const std::string solver_name, Problem *P,
                                  bool verbose, int argc, char *argv[])
{
  std::unique_ptr<Solver> solver;
  if (solver_name == "NetworkFlow") {
    solver = std::make_unique<NetworkFlow>(P);
  } else if (solver_name == "NaiveTSWAP") {
    solver = std::make_unique<NaiveTSWAP>(P);
  } else if (solver_name == "TSWAP") {
    solver = std::make_unique<TSWAP>(P);
  } else {
    warn("unknown solver name, " + solver_name + ", continue by PIBT");
    solver = std::make_unique<TSWAP>(P);
  }
  solver->setParams(argc, argv);
  solver->setVerbose(verbose);
  return solver;
}

void printHelp()
{
  std::cout << "\nUsage: ./app [OPTIONS] [SOLVER-OPTIONS]\n"
            << "\n**instance file is necessary to run MAPF simulator**\n\n"
            << "  -i --instance [FILE_PATH]     instance file path\n"
            << "  -o --output [FILE_PATH]       ouptut file path\n"
            << "  -v --verbose                  print additional info\n"
            << "  -h --help                     help\n"
            << "  -s --solver [SOLVER_NAME]     solver, choose from the below\n"
            << "  -P --make-scen                make scenario file using "
               "random starts/goals"
            << "\n\nSolver Options:" << std::endl;
  // each solver
  NetworkFlow::printHelp();
  NaiveTSWAP::printHelp();
  TSWAP::printHelp();
}
