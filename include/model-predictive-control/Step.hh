#pragma once

#include <iostream>
#include <Eigen/Core>

namespace mpc
{
class Step
{
 public:
  Step(double x, double y, double tMin, double tMax)
      : x_(x), y_(y), tMin_(tMin), tMax_(tMax)
  {
  }
  virtual ~Step(){};

  const double& x() const {return x_;}
  const double& y() const {return y_;}
  const double& tMin() const {return tMin_;}
  const double& tMax() const {return tMax_;}

 private:
  double x_;
  double y_;
  double tMin_;
  double tMax_;
};

} /* mpc */
