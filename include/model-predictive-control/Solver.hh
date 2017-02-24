#pragma once

#include <iostream>
#include <model-predictive-control/Problem.hh>
#include <model-predictive-control/Controller.hh>

namespace mpc
{
  class Solver
  {
   public:
    Solver(const long& nHorizon, const long& nTotal);
    virtual ~Solver();
    void solve(const Problem& prob, Controller& ctrl);
    const Eigen::VectorXd& jerk() const { return jerk_; };
    const Eigen::MatrixXd& yStateHistory() const { return yStateHistory_; };
    const Eigen::MatrixXd& copHistory() const { return copHistory_; };

  private:
    Eigen::VectorXd jerk_;
    Eigen::MatrixXd yStateHistory_;
    Eigen::MatrixXd copHistory_;
  };
} /* mpc */ 
