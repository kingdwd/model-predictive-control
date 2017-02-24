#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>

#include <model-predictive-control/Step.hh>
#include <model-predictive-control/StepPlan.hh>
#include <model-predictive-control/Controller.hh>
#include <model-predictive-control/logger.hh>
#include <model-predictive-control/Problem.hh>
#include <model-predictive-control/Solver.hh>

using namespace mpc;

int main(int argc, char* argv[])
{
  assert(argc >= 2 && "Need to provide a config file");

  std::string ymlPath = std::string(CONFIGS_DATA_DIR) + "/" + argv[1] + ".yml";
  Problem prob(ymlPath);
  Controller controller(prob);
  Solver solver(prob.nHorizon(), prob.nTotal());
  solver.solve(prob, controller);
  logResult("results.py", prob, solver.yStateHistory(), solver.copHistory());

  std::string command = "python results.py";
  system(command.c_str());
  return 0;
}
