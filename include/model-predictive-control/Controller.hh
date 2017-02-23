#pragma once

#include <Eigen/Core>
#include <EigenQP/LSSOL_QP.h>

namespace mpc
{
class Controller
{
 public:
  Controller(const double& timeStep, const double& h_CoM, const double& g,
             const long& nHorizon);
  void solve(Eigen::VectorXd& res, const Eigen::Vector3d& x_state,
             const Eigen::VectorXd& y_min, const Eigen::VectorXd& y_max);

  virtual ~Controller(){}

 private:
  double T_ = 0.005;
  double h_CoM_ = 0.8;
  double g_ = 9.81;
  long N_ = 10;
  Eigen::LSSOL_QP QPSolver_;

  //Matrices to feed to QPSolver
  Eigen::MatrixXd A_;
  Eigen::MatrixXd C_;
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;

  //Matrices to compute CoP
  Eigen::MatrixXd stateM_; //multiplies the state vector
  Eigen::MatrixXd jerkM_; //multiplies the jerk vector
  Eigen::VectorXd jerkLB_; //Lower bound for jerk
  Eigen::VectorXd jerkUB_; //Upper bound for jerk
};

} /* mpc */

