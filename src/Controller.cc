#include <iostream>
#include <model-predictive-control/Controller.hh>

namespace mpc
{
Controller::Controller(const double& timeStep, const double& h_CoM,
                       const double& g, const long& nHorizon)
    : T_(timeStep),
      h_CoM_(h_CoM),
      g_(g),
      N_(nHorizon),
      A_(N_, N_),
      C_(N_, N_),
      lb_(N_),
      ub_(N_),
      stateM_(N_, 3),
      jerkM_(N_, N_),
      jerkLB_(N_),
      jerkUB_(N_)
{
  bool warm = true;
  double crashTol = 1e-2;
  double feasTol = std::sqrt(std::numeric_limits<double>::epsilon());
  double infiniteBnd = 1e10;
  double infiniteStep = 1e10;
  int feasMaxIter = 1000;
  int optimMaxIter = 1000;
  int printLevel = 0;
  Eigen::lssol::eType type = Eigen::lssol::QP1;
  double rankTol = std::sqrt(std::numeric_limits<double>::epsilon());

  QPSolver_.resize(static_cast<int>(N_), static_cast<int>(N_), type);
  QPSolver_.setAllLSSOLParam(warm, crashTol, feasTol, infiniteBnd, infiniteStep,
                             feasMaxIter, optimMaxIter, printLevel, type,
                             rankTol);
  A_.setIdentity();
  C_.setZero();
  stateM_.setZero();
  jerkM_.setZero();
  
  double inf = std::numeric_limits<double>::infinity();

  for (long i = 0; i < N_; i++) 
  {
    stateM_.row(i) << 1, (i + 1) * T_,
        (i + 1) * (i + 1) * T_ * T_ / 2.0 - h_CoM_ / g_;
    double diagVal =
        (1 + 3 * i + 3 * i * i) * T_ * T_ * T_ / 6.0 - T_ * h_CoM_ / g_;
    jerkLB_(i) = -inf;
    jerkUB_(i) = inf;
    for (long j = 0; j < N_ - i; j++)
    {
      jerkM_(j + i, j) = diagVal;
    }
  }
}

void Controller::solve(Eigen::VectorXd& res, const Eigen::Vector3d& x_state,
                       const Eigen::VectorXd& y_min,
                       const Eigen::VectorXd& y_max)
{
  A_.setIdentity();
  //std::cout << "A_: " << A_ << std::endl;
  //QPSolver_.print(jerkLB_, jerkUB_, A_, jerkM_, y_min - stateM_ * x_state,
                  //y_max - stateM_ * x_state);
  QPSolver_.solve(jerkLB_, jerkUB_, A_, jerkM_, y_min - stateM_ * x_state,
                  y_max - stateM_ * x_state);
  res = QPSolver_.result();
}

} /* mpc */
