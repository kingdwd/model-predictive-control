#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>

#include <model-predictive-control/Step.hh>
#include <model-predictive-control/StepPlan.hh>
#include <model-predictive-control/Controller.hh>

using namespace mpc;

int main(void)
{
  double T = 0.005;  // timeStep
  double h_CoM = 0.8;
  double g = 9.81;
  long nHorizon = 20;

  std::vector<Step> leftSteps;
  std::vector<Step> rightSteps;

  leftSteps.push_back(Step(0, 0.1, 0, 2.0));
  rightSteps.push_back(Step(0, -0.1, 0, 3.1));

  leftSteps.push_back(Step(0.2, 0.1, 2.9, 4.1));
  rightSteps.push_back(Step(0.4, -0.1, 3.9, 5.1));

  leftSteps.push_back(Step(0.6, 0.1, 4.9, 6.1));
  rightSteps.push_back(Step(0.8, -0.1, 5.9, 10.0));

  leftSteps.push_back(Step(0.8, 0.1, 6.9, 10.0));

  StepPlan stepPlan(leftSteps, rightSteps, T);

  Controller controller(T, h_CoM, g, nHorizon);

  Eigen::VectorXd jerk(nHorizon);
  jerk.setZero();

  long nTotal = stepPlan.tMax() / T;
  Eigen::Vector3d yState(0, 0, 0);
  Eigen::MatrixXd yStateHistory(3, nTotal);
  Eigen::MatrixXd copHistory(1, nTotal);
  yStateHistory.col(0) << yState;

  Eigen::Matrix3d stateIntegrator;
  Eigen::Vector3d jerkIntegrator;
  stateIntegrator << 1, T, T* T / 2.0, 0, 1, T, 0, 0, 1;
  jerkIntegrator << T* T* T / 6, T* T / 2, T;

  int k = 0;
  while ((k + nHorizon + 1) * T < stepPlan.tMax())
  {
    std::cout << "\nResolution for time = " << k* T << "s" << std::endl;
    std::cout << "(k+nHorizon)*T: " << (k+nHorizon)*T << std::endl;
    std::cout << "stepPlan.tMax(): " << stepPlan.tMax() << std::endl;

    controller.solve(jerk, yState, stepPlan.y_min().segment(k, nHorizon),
                     stepPlan.y_max().segment(k, nHorizon));
    yState = stateIntegrator * yState + jerkIntegrator * jerk(0);

    k++;
    yStateHistory.col(k) << yState;
  }
  std::cout << "done" << std::endl;

  copHistory = Eigen::Vector3d(1, 0, -h_CoM / g).transpose() * yStateHistory;

  Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
  std::ofstream logFile("results.py");
  logFile << "import numpy as np\nimport matplotlib.pyplot as plt\nt = "
             "np.arange(0.," << stepPlan.tMax() << ", " << T << ")\n\n";

  logFile << "y_min = np.array("
          << stepPlan.y_min().transpose().format(cleanFmt) << ")" << std::endl;
  logFile << "y_max = np.array("
          << stepPlan.y_max().transpose().format(cleanFmt) << ")" << std::endl;
  logFile << "y = np.array(" << yStateHistory.row(0).format(cleanFmt) << ")"
          << std::endl;
  logFile << "yVel = np.array(" << yStateHistory.row(1).format(cleanFmt) << ")"
          << std::endl;
  logFile << "p = np.array(" << copHistory.format(cleanFmt) << ")" << std::endl;

  logFile << "plt.plot(t, y_min, 'r--', label='y_min')\n";
  logFile << "plt.plot(t, y_max, 'b--', label='y_max')\n";
  logFile << "plt.plot(t, y, 'g^', label='y')\n";
  logFile << "plt.plot(t, p, 'ko', label='CoP')\n";
  logFile << "legend = plt.legend(loc='upper center', shadow=True)\n";
  logFile << "plt.axis([0, 10, -0.3, 0.3])\nplt.show()\n";
  logFile.close();

  std::string command = "python results.py";
  system(command.c_str());
  return 0;
}
