#include "kalman_filter.h"

using namespace carnd_ekf;

KalmanFilter::KalmanFilter() {
  I_ = Eigen::MatrixXd::Identity(2, 2);
}

void KalmanFilter::init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in,
                        Eigen::MatrixXd& F_in, Eigen::MatrixXd& H_in,
                        Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::predict() {
  // prior
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
  // residual
  Eigen::VectorXd y = z - H_ * x_;
  // subexpression
  Eigen::MatrixXd PHT = P_ * H_.transpose();
  // system uncertainity
  Eigen::MatrixXd S = H_ * PHT + R_;
  // kalman gain
  Eigen::MatrixXd K = PHT * S.inverse();
  // posterior
  x_ = x_ + (K * y);

  /*
   * P = (I - KH) * P is numerically unstable because (I - KH) float error can
   * make P non-symmetric
   * equivalent but stable derivation P = (I-KH)P(I-KH)' + KRK'
   */
  Eigen::MatrixXd I_KH = I_ - K * H_;
  P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();
}
