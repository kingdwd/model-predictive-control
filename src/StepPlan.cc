#include <model-predictive-control/StepPlan.hh>

namespace mpc
{
StepPlan::StepPlan() {}
StepPlan::StepPlan(std::vector<Step> leftSteps, std::vector<Step> rightSteps,
                   double T)
    : leftSteps_(leftSteps), rightSteps_(rightSteps), T_(T)
{
  computePlan(leftSteps_, rightSteps_, T_);
}

void StepPlan::computePlan(std::vector<Step> leftSteps,
                           std::vector<Step> rightSteps, double T)
{
  leftSteps_ = leftSteps;
  rightSteps_ = rightSteps;
  T_ = T;
  tMax_ = leftSteps[0].tMax();
  stepYWidth_ = 0.1;

  for (auto step : leftSteps_)
    if (step.tMax() > tMax_) tMax_ = step.tMax();

  for (auto step : rightSteps_)
    if (step.tMax() > tMax_) tMax_ = step.tMax();

  int nTimeSteps = tMax_ / T_;
  Eigen::VectorXd time(nTimeSteps);
  for (long i = 0; i < nTimeSteps; i++)
  {
    time(i) = i * T;
  }

  y_min_.resize(nTimeSteps);
  y_max_.resize(nTimeSteps);
  y_min_.setZero();
  y_max_.setZero();

  for (long iTimeStep = 0; iTimeStep < nTimeSteps; iTimeStep++)
  {
    bool singleSupportLeft = false;
    bool singleSupportRight = false;
    bool doubleSupport = false;
    double currentTime = time(iTimeStep);

    size_t iStepLeft = 0;
    size_t iStepRight = 0;

    for (size_t i = 0; i < leftSteps_.size(); i++)
    {
      if (leftSteps_[i].tMin() <= currentTime &&
          currentTime < leftSteps_[i].tMax())
      {
        singleSupportLeft = true;
        iStepLeft = i;
      }
    }

    for (size_t i = 0; i < rightSteps_.size(); i++)
    {
      if (rightSteps_[i].tMin() <= currentTime &&
          currentTime < rightSteps_[i].tMax())
      {
        singleSupportRight = true;
        iStepRight = i;
      }
    }

    if (singleSupportLeft && singleSupportRight)
    {
      singleSupportLeft = false;
      singleSupportRight = false;
      doubleSupport = true;
    }

    if (singleSupportLeft)
    {
      y_min_[iTimeStep] = leftSteps_[iStepLeft].y() - stepYWidth_ / 2;
      y_max_[iTimeStep] = leftSteps_[iStepLeft].y() + stepYWidth_ / 2;
    }
    else if (singleSupportRight)
    {
      y_min_[iTimeStep] = rightSteps_[iStepRight].y() - stepYWidth_ / 2;
      y_max_[iTimeStep] = rightSteps_[iStepRight].y() + stepYWidth_ / 2;
    }
    else if (doubleSupport)
    {
      y_min_[iTimeStep] = rightSteps_[iStepRight].y() - stepYWidth_ / 2;
      y_max_[iTimeStep] = leftSteps_[iStepLeft].y() + stepYWidth_ / 2;
    }
  }
}
} /* mpc */
