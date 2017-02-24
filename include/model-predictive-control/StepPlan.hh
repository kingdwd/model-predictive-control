#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Core>

#include <model-predictive-control/Step.hh>

namespace mpc
{
class StepPlan
{
 public:
  StepPlan();
  StepPlan(std::vector<Step> leftSteps, std::vector<Step> rightSteps, double T);
  void computePlan(std::vector<Step> leftSteps, std::vector<Step> rightSteps,
                   double T);
  virtual ~StepPlan(){};
  const Eigen::VectorXd& y_min() const { return y_min_; }
  const Eigen::VectorXd& y_max() const { return y_max_; }
  const double& tMax() const { return tMax_; }

 private:
  std::vector<Step> leftSteps_;
  std::vector<Step> rightSteps_;
  // Eigen::VectorXd x_min_;
  // Eigen::VectorXd x_max_;
  Eigen::VectorXd y_min_;
  Eigen::VectorXd y_max_;
  double tMax_;
  double T_;
  // double stepXWidth_;
  double stepYWidth_;
};
} /* mpc */
