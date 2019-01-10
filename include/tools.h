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

  /*
   * read data file and extract measurements and ground truth
   * @param file_name name of file
   * @param measurements vector of measurements
   * @param ground_truth vector of ground truth
   */
  bool read_data_file(const std::string& file_name,
    std::vector<carnd_ekf::MeasurementPackage>& measurements,
    std::vector<Eigen::VectorXd>& ground_truth);

  /*
   * write estimations to csv file ('\t')
   * format px py vx vy
   * @param file_name output file name
   * @param estimation vector of estimations (x_)
   */
  void write_output_csv(const std::string& file_name,
                        const std::vector<Eigen::VectorXd> &estimations);

  /*
   * calculate root mean square error between estimations and ground truth
   * @param estimations vector of estimations vectors
   * @param ground_truth vector of ground truth vectors
   */
  Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                const std::vector<Eigen::VectorXd> &ground_truth);

} // namespace carnd_ekf

#endif // _EKF_TOOLS_H_
