#include "tools.h"

Eigen::VectorXd carnd_ekf::calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                         const std::vector<Eigen::VectorXd> &ground_truth) {
  Eigen::VectorXd rmse(4); rmse << 0,0,0,0;
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }
  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {
    Eigen::VectorXd residual = estimations[i] - ground_truth[i];
    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  // calculate the mean
  rmse = rmse/estimations.size();
  // calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}
