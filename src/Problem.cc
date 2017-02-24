#include <model-predictive-control/Problem.hh>

namespace mpc
{
Problem::Problem(const std::string& configFile)
    : configFile_(configFile), config_(YAML::LoadFile(configFile_))
{
  T_ = config_["T"].as<double>();
  h_CoM_ = config_["h_CoM"].as<double>();
  g_ = config_["g"].as<double>();
  nHorizon_ = config_["nHorizon"].as<long>();

  for (int i = 0; i < 3; i++)
    initState_[i] = config_["initState"][i].as<double>();

  stateIntegrator_ << 1, T_, T_* T_ / 2.0, 0, 1, T_, 0, 0, 1;
  jerkIntegrator_ << T_* T_* T_ / 6, T_* T_ / 2, T_;

  readSteps();

  stepPlan_.computePlan(leftSteps_, rightSteps_, T_);
  nTotal_ = static_cast<long>(std::floor(stepPlan_.tMax() / T_));
}

void Problem::readSteps()
{
  for (auto subNode : config_["steps"])
  {
    std::string side;
    double x, y, tmin, tmax;
    bool hasSide = false;
    bool hasX = false;
    bool hasY = false;
    bool hasTMin = false;
    bool hasTMax = false;
    for (auto i : subNode)
    {
      if (std::string("side").compare(i.first.as<std::string>()) == 0)
      {
        side = i.second.as<std::string>();
        hasSide = true;
      }
      else if (std::string("x").compare(i.first.as<std::string>()) == 0)
      {
        x = i.second.as<double>();
        hasX = true;
      }
      else if (std::string("y").compare(i.first.as<std::string>()) == 0)
      {
        y = i.second.as<double>();
        hasY = true;
      }
      else if (std::string("tmin").compare(i.first.as<std::string>()) == 0)
      {
        tmin = i.second.as<double>();
        hasTMin = true;
      }
      else if (std::string("tmax").compare(i.first.as<std::string>()) == 0)
      {
        tmax = i.second.as<double>();
        hasTMax = true;
      }
    }
    if (hasSide && hasX && hasY && hasTMax && hasTMin)
    {
      if (side.compare("L") == 0)
        leftSteps_.push_back(Step(x, y, tmin, tmax));
      else if (side.compare("R") == 0)
        rightSteps_.push_back(Step(x, y, tmin, tmax));
    }
    else
    {
      std::cerr << "Warning: you have ill formed steps" << std::endl;
    }
  }
}

} /* mpc */
