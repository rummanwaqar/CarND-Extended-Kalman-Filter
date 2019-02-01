#include "ukf.h"

#include <iostream>
using namespace carnd_ekf;
//
Ukf::Ukf(SigmaPoints& points) {
  Eigen::VectorXd x = Eigen::VectorXd(4); x << 1, 2, 3, 4;
  Eigen::VectorXd initial_var(4); initial_var << 1., 1000., 1., 1000.;
  Eigen::MatrixXd P = initial_var.asDiagonal();
  std::cout << points.get_sigma_points(x, P) << std::endl;
}
//
Ukf::~Ukf() {

}

void Ukf::predict(double dt) {

  // compute sigma points with weights

  // calculate prior

  // calculate mean and variance of prior using unscented transform

}
