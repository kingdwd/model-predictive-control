#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>

#include <Eigen/Core>

#include <model-predictive-control/Problem.hh>
#include <model-predictive-control/logger.hh>

namespace mpc
{
void logResult(const std::string& logFileName, const Problem& prob,
               const Eigen::MatrixXd& yStateHistory,
               const Eigen::MatrixXd& copHistory)
{
  Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
  std::ofstream logFile(logFileName);
  logFile << "import numpy as np\n"
          << "import matplotlib.pyplot as plt\n"
          << "t = np.arange(0.," << prob.stepPlan().tMax() << ", " << prob.T()
          << ")\n\n";

  logFile << "y_min = np.array("
          << prob.stepPlan().y_min().transpose().format(cleanFmt) << ")\n";
  logFile << "y_max = np.array("
          << prob.stepPlan().y_max().transpose().format(cleanFmt) << ")\n";
  logFile << "y = np.array(" << yStateHistory.row(0).format(cleanFmt) << ")\n";
  logFile << "yVel = np.array(" << yStateHistory.row(1).format(cleanFmt)
          << ")\n";
  logFile << "p = np.array(" << copHistory.format(cleanFmt) << ")\n";

  logFile << "plt.plot(t, y_min, 'r--', label='y_min')\n";
  logFile << "plt.plot(t, y_max, 'b--', label='y_max')\n";
  logFile << "plt.plot(t, y, 'g^', label='y')\n";
  logFile << "plt.plot(t, p, 'ko', label='CoP')\n";
  logFile << "legend = plt.legend(loc='upper center', shadow=True)\n";
  logFile << "plt.axis('auto')\n";
  logFile << "plt.show()\n";
  logFile.close();
}
} /* mpc */
