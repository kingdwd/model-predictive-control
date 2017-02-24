#pragma once

namespace mpc
{
void logResult(const std::string& logFileName, const Problem& prob,
               const Eigen::MatrixXd& yStateHistory,
               const Eigen::MatrixXd& copHistory);
} /* mpc */
