#ifndef _EKF_KALMAN_FILTER_H_
#define _EKF_KALMAN_FILTER_H_

#include <functional>
#include <cmath>
#include "Eigen/Dense"

namespace carnd_ekf {
  class KalmanFilter {
  public:
    /*
     * Constructor
     */
    KalmanFilter();

    /*
     * Destructor
     */
    virtual ~KalmanFilter() = default;

  protected:
    /*
     * predicts the state and state covariance using the process model
     */
    void predict();

    /*
     * updates the state using measurement and kalman equations
     * @param z = measurement
     * @param h = measurement function (optional only for EKF)
     */
    void update(const Eigen::VectorXd& z,
                std::function<Eigen::VectorXd (const Eigen::VectorXd&)> h = nullptr);

  protected:
    // state vector
    Eigen::VectorXd x_;

    // state covariance
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance
    Eigen::MatrixXd R_;

  private:
    // identity matrix
    Eigen::MatrixXd I_;
  }; // class KalmanFilter
} // namespace carnd_ekf

#endif  // _EKF_KALMAN_FILTER_H_
