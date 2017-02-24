#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>

#include <model-predictive-control/Step.hh>
#include <model-predictive-control/StepPlan.hh>
#include <model-predictive-control/Controller.hh>
#include <model-predictive-control/logger.hh>
#include <model-predictive-control/Problem.hh>

using namespace mpc;

int main(int argc, char* argv[])
{
  assert(argc >= 2 && "Need to provide a config file");

  std::string ymlPath = std::string(CONFIGS_DATA_DIR) + "/" + argv[1] + ".yml";
  Problem prob(ymlPath);
  Controller controller(prob);

  Eigen::VectorXd jerk(prob.nHorizon());
  jerk.setZero();

  Eigen::Vector3d yState(0, 0, 0);
  Eigen::MatrixXd yStateHistory(3, prob.nTotal());
  Eigen::MatrixXd copHistory(1, prob.nTotal());
  yStateHistory.col(0) << yState;

  int k = 0;
  while ((k + prob.nHorizon() + 1) * prob.T() < prob.stepPlan().tMax())
  {
    std::cout << "Resolution for time = " << k* prob.T() << "s" << std::endl;

    controller.solve(jerk, yState,
                     prob.stepPlan().y_min().segment(k, prob.nHorizon()),
                     prob.stepPlan().y_max().segment(k, prob.nHorizon()));
    yState = prob.stateIntegrator() * yState + prob.jerkIntegrator() * jerk(0);

    k++;
    yStateHistory.col(k) << yState;
  }

  copHistory = Eigen::Vector3d(1, 0, -prob.h_CoM() / prob.g()).transpose() *
               yStateHistory;

  logResult("results.py", prob, yStateHistory, copHistory);

  std::string command = "python results.py";
  system(command.c_str());
  return 0;
}
