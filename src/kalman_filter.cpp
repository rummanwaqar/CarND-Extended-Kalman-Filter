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
    while ( y(1) > M_PI || y(1) < -M_PI ) {
      if ( y(1) > M_PI ) {
        y(1) -= M_PI;
      } else {
        y(1) += M_PI;
      }
    }
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

Eigen::MatrixXd KalmanFilter::calculate_jacobian(const Eigen::VectorXd& x_state) {
  Eigen::MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float vx = x_state(1);
  float py = x_state(2);
  float vy = x_state(3);
  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  // check division by zero
  if (fabs(c1) < 0.0001) {
    throw std::runtime_error("Division by zero error");
  }
  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  return Hj;
}
