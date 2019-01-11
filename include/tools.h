#ifndef _EKF_TOOLS_H_
#define _EKF_TOOLS_H_

#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Dense"

#include "measurement_package.h"

namespace carnd_ekf {
  /*
   * calculate root mean square error between estimations and ground truth
   * @param estimations vector of estimations vectors
   * @param ground_truth vector of ground truth vectors
   */
  Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                const std::vector<Eigen::VectorXd> &ground_truth);

} // namespace carnd_ekf

#endif // _EKF_TOOLS_H_
