#ifndef _EKF_KALMAN_FILTER_H_
#define _EKF_KALMAN_FILTER_H_

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
     */
    void update(const Eigen::VectorXd& z);

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
