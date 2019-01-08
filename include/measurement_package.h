#ifndef _EKF_MEASUREMENT_PACKAGE_H_
#define _EKF_MEASUREMENT_PACKAGE_H_

#include <string>
#include <string>
#include <vector>
#include <cstdlib>
#include "Eigen/Dense"

namespace carnd_ekf {
  class MeasurementPackage {
  public:
    enum SensorType {
      LIDAR,
      RADAR
    } sensor_type;

    long long timestamp;

    Eigen::VectorXd raw_measurements;

    MeasurementPackage() {}
    MeasurementPackage(std::vector<std::string>& tokens) {
      if(tokens[0] == "R") {
        // radar measurement
        sensor_type = carnd_ekf::MeasurementPackage::RADAR;
        raw_measurements = Eigen::VectorXd(3);
        raw_measurements <<
          std::stod(tokens[1].c_str()), // rho
          std::stod(tokens[2].c_str()), // phi
          std::stod(tokens[3].c_str()); // rhodot
        timestamp = std::stod(tokens[4].c_str());
      } else if (tokens[0] == "L") {
        // lidar measurement
        sensor_type = carnd_ekf::MeasurementPackage::LIDAR;
        raw_measurements = Eigen::VectorXd(2);
        raw_measurements <<
          std::stod(tokens[1].c_str()), // px
          std::stof(tokens[2].c_str()); // py
        timestamp = std::stod(tokens[3].c_str());
      }
    }
  };
} // namespace carnd_ekf

#endif // _EKF_MEASUREMENT_PACKAGE_H_
