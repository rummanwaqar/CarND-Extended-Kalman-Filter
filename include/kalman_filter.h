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

    /*
     * Initialize kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in, Eigen::MatrixXd& F_in,
              Eigen::MatrixXd& H_in, Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in);

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
