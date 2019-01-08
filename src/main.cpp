#include <iostream>
#include <string>
#include <vector>

#include "measurement_package.h"
#include "kalman_filter.h"
#include "tools.h"

std::string FILE_NAME = "../data/obj_pose-laser-radar-synthetic-input.txt";

int main() {
  // std::vector<carnd_ekf::MeasurementPackage> measurements;
  // std::vector<Eigen::VectorXd> ground_truth;
  //
  // assert(carnd_ekf::read_data_file(FILE_NAME, std::ref(measurements), std::ref(ground_truth)));
  // assert(measurements.size() == ground_truth.size());

  // test
  Eigen::VectorXd x;	// object state
  Eigen::MatrixXd P;	// object covariance matrix
  Eigen::VectorXd u;	// external motion
  Eigen::MatrixXd F; // state transition matrix
  Eigen::MatrixXd H;	// measurement matrix
  Eigen::MatrixXd R;	// measurement covariance matrix
  Eigen::MatrixXd I; // Identity matrix
  Eigen::MatrixXd Q;	// process covariance matrix
  x = Eigen::VectorXd(2);
  x << 0, 0;

  P = Eigen::MatrixXd(2, 2);
  P << 1000, 0, 0, 1000;

  u = Eigen::VectorXd(2);
  u << 0, 0;

  F = Eigen::MatrixXd(2, 2);
  F << 1, 1, 0, 1;

  H = Eigen::MatrixXd(1, 2);
  H << 1, 0;

  R = Eigen::MatrixXd(1, 1);
  R << 1;

  I = Eigen::MatrixXd::Identity(2, 2);

  Q = Eigen::MatrixXd(2, 2);
  Q << 0, 0, 0, 0;

  // create a list of measurements
  std::vector<Eigen::VectorXd> measurements;
  Eigen::VectorXd single_meas(1);
  single_meas << 1;
  measurements.push_back(single_meas);
  single_meas << 2;
  measurements.push_back(single_meas);
  single_meas << 3;
  measurements.push_back(single_meas);

  carnd_ekf::KalmanFilter filter;
  filter.init(x, P, F, H, R, Q);

  for(auto const& z : measurements) {
    filter.update(z);
    filter.predict();
  }
  std::cout << filter.x_ << "\n" << filter.P_ << std::endl;

}
