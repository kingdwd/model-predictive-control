#include <model-predictive-control/testClass.hh>

namespace mpc
{
testClass::testClass(double a) : a_(a)
{
  std::cout << "testClass ctor with a=" << a_ << std::endl;
  bool warm = true;
  double crashTol = 1e-2;
  double feasTol = std::sqrt(std::numeric_limits<double>::epsilon());
  double infiniteBnd = 1e10;
  double infiniteStep = 1e10;
  int feasMaxIter = 1000;
  int optimMaxIter = 1000;
  int printLevel = 0;
  Eigen::lssol::eType type = Eigen::lssol::LS1;
  double rankTol = std::sqrt(std::numeric_limits<double>::epsilon());

  QPSolver_.resize(20, 20, Eigen::lssol::eType::QP4);
  QPSolver_.setAllLSSOLParam(warm, crashTol, feasTol, infiniteBnd, infiniteStep,
                             feasMaxIter, optimMaxIter, printLevel, type,
                             rankTol);
  
}
testClass::~testClass() {}

} /* mpc */
