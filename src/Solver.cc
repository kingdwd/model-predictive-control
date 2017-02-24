#include <model-predictive-control/Solver.hh>

namespace mpc
{
Solver::Solver(const long& nHorizon, const long& nTotal)
    : jerk_(nHorizon), yStateHistory_(3, nTotal), copHistory_(1, nTotal)
{
}

Solver::~Solver() {}

void Solver::solve(const Problem& prob, Controller& ctrl)
{
  jerk_.setZero();
  Eigen::Vector3d yState(prob.initState());

  yStateHistory_.col(0) << yState;

  int k = 0;
  while ((k + prob.nHorizon() + 1) * prob.T() < prob.stepPlan().tMax())
  {
    std::cout << "Resolution for time = " << k* prob.T() << "s" << std::endl;

    ctrl.solve(jerk_, yState,
               prob.stepPlan().y_min().segment(k, prob.nHorizon()),
               prob.stepPlan().y_max().segment(k, prob.nHorizon()));
    yState = prob.stateIntegrator() * yState + prob.jerkIntegrator() * jerk_(0);

    k++;
    yStateHistory_.col(k) << yState;
  }

  copHistory_ = Eigen::Vector3d(1, 0, -prob.h_CoM() / prob.g()).transpose() *
                yStateHistory_;
}
} /* mpc */
