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

  const double& T() const { return T_; };
  const double& h_CoM() const { return h_CoM_; }
  const double& g() const { return g_; };
  const long& nHorizon() const { return nHorizon_; }
  const long& nTotal() const { return nTotal_; }
  const std::vector<Step>& leftSteps() const { return leftSteps_; };
  const std::vector<Step>& rightSteps() const { return rightSteps_; };
  const StepPlan& stepPlan() const { return stepPlan_; };
  const Eigen::Matrix3d stateIntegrator() const { return stateIntegrator_; };
  const Eigen::Vector3d jerkIntegrator() const { return jerkIntegrator_; };
  const Eigen::Vector3d initState() const { return initState_; };

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
  Eigen::Vector3d initState_;
  Eigen::Matrix3d stateIntegrator_;
  Eigen::Vector3d jerkIntegrator_;
};

} /* model-predictive-control */
