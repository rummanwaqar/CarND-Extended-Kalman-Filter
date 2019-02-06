#include "ukf.h"
#include <iostream>

using namespace carnd_ekf;

Ukf::Ukf(int n,
  SigmaPoints&& points,
  std::function<Eigen::VectorXd (const Eigen::VectorXd&, double)> fx,
  std::function<Eigen::VectorXd (const Eigen::VectorXd&, const Eigen::VectorXd&)> residual_fn,
  std::function<Eigen::VectorXd (const Eigen::MatrixXd&, const Eigen::VectorXd&)> mean_fn)
  : points_(points), fx_(fx), residual_fn_(residual_fn), mean_fn_(mean_fn), x_(n) {
    x_ << 1, 2, 3, 4;
    Eigen::VectorXd initial_var(4); initial_var << 1., 1000., 1., 1000.;
    P_ = initial_var.asDiagonal();
  }

Ukf::~Ukf() {

}

void Ukf::predict(double dt) {
  // get sigma points
  Eigen::MatrixXd sigmas = points_.get_sigma_points(x_, P_);
  // calculate prior
  for(int i=0; i<sigmas.cols(); i++) {
    sigmas.col(i) = fx_(sigmas.col(i), dt);
  }
  // calculate mean and variance of prior using unscented transform
  std::tie(x_, P_) = unscented_transform(sigmas, points_.get_Wm(), points_.get_Wm());
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> Ukf::unscented_transform(
  Eigen::MatrixXd& sigmas, Eigen::VectorXd Wm, Eigen::VectorXd Wc) {
  int n = sigmas.rows();
  // calculate mean
  Eigen::VectorXd x(n);
  if(mean_fn_ != nullptr) {
    x = mean_fn_(sigmas, Wm);
  } else {
    x.fill(0.0);
    for(int i=0; i<Wm.size(); i++) {
      x += Wm(i) * sigmas.col(i);
    }
  }
  // calculate covariances
  Eigen::MatrixXd P(n,n);
  P.fill(0.0);
  for(int i=0; i<Wc.size(); i++) {
    Eigen::VectorXd y;
    if(residual_fn_ != nullptr) {
      y = residual_fn_(sigmas.col(i), x);
    } else {
      y = sigmas.col(i) - x;
    }
    P += Wc(i) * y * y.transpose();
  }
  return std::make_tuple(x, P);
}
