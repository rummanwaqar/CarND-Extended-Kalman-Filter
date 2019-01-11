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
  };
} // namespace carnd_ekf

#endif // _EKF_MEASUREMENT_PACKAGE_H_
