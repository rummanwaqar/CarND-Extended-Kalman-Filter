#include "kalman_filter.h"
#include <iostream>

using namespace carnd_ekf;

KalmanFilter::KalmanFilter() {
  I_ = Eigen::MatrixXd::Identity(4, 4);
}

void KalmanFilter::predict() {
  // prior
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z,
                          std::function<Eigen::VectorXd (const Eigen::VectorXd&)> h) {
  // residual
  Eigen::VectorXd y;
  if(h != nullptr) { // extended kalman filter
    y = z - h(x_);
    // normalize ϕ:y(1) around -π to π
    y(1) = std::atan2(std::sin(y(1)), std::cos(y(1)));
  } else { // kalman filter
    y = z - H_ * x_;
  }
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
