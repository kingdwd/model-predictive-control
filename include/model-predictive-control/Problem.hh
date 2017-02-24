#pragma once

#include <iostream>

#include <yaml-cpp/yaml.h>
#include <model-predictive-control/Step.hh>
#include <model-predictive-control/StepPlan.hh>

namespace mpc
{
class Problem
{
 public:
  Problem(const std::string& configFile = "");
  virtual ~Problem(){};
  void readSteps();

  const double& T() const;
  const double& h_CoM() const;
  const double& g() const;
  const long& nHorizon() const;
  const long& nTotal() const;
  const std::vector<Step>& leftSteps() const;
  const std::vector<Step>& rightSteps() const;
  const StepPlan& stepPlan() const;
  const Eigen::Matrix3d stateIntegrator() const;
  const Eigen::Vector3d jerkIntegrator() const;

 private:
  std::string configFile_;
  YAML::Node config_;
  double T_;
  double h_CoM_;
  double g_;
  long nHorizon_;
  long nTotal_;
  std::vector<Step> leftSteps_;
  std::vector<Step> rightSteps_;
  StepPlan stepPlan_;
  Eigen::Matrix3d stateIntegrator_;
  Eigen::Vector3d jerkIntegrator_;
};

} /* model-predictive-control */
