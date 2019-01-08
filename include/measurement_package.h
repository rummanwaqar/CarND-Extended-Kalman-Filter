#ifndef _EKF_MEASUREMENT_PACKAGE_H_
#define _EKF_MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

namespace carnd_ekf {
  class MeasurementPackage {
  public:
    enum SensorType {
      LASER,
      RADAS
    } sensor_type;

    long long timestamp;

    Eigen::VectorXd raw_measurements;
  };
} // namespace carnd_ekf

#endif // _EKF_MEASUREMENT_PACKAGE_H_
