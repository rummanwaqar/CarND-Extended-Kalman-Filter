#ifndef _EKF_TOOLS_H_
#define _EKF_TOOLS_H_

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include "Eigen/Dense"

#include "measurement_package.h"

namespace carnd_ekf {
  /*
   * split a line by tabs and return a vector or strings
   * @param line to parse
   * @tokens return vector of strings by reference
   */
  void split_by_tab(const std::string&& line, std::vector<std::string>& tokens);

  /*
   * convert string tokens into a vector for ground truth
   * @param vector of tokens
   * @returns Eigen vector containing ground truth data
   */
  Eigen::VectorXd get_ground_truth(std::vector<std::string>& tokens);

  bool read_data_file(const std::string& file_name,
    std::vector<carnd_ekf::MeasurementPackage>& measurements,
    std::vector<Eigen::VectorXd>& ground_truth);

  Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                const std::vector<Eigen::VectorXd> &ground_truth);

} // namespace carnd_ekf

#endif // _EKF_TOOLS_H_
