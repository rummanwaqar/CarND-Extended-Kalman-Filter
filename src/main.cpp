#include <iostream>
#include <string>
#include <vector>

#include "measurement_package.h"
#include "fusion.h"
#include "kalman_filter.h"
#include "tools.h"

std::string FILE_NAME = "../data/obj_pose-laser-radar-synthetic-input.txt";

int main() {
  carnd_ekf::Fusion fusion;
  std::vector<carnd_ekf::MeasurementPackage> measurements;
  std::vector<Eigen::VectorXd> ground_truth;

  assert(carnd_ekf::read_data_file(FILE_NAME, std::ref(measurements), std::ref(ground_truth)));
  assert(measurements.size() == ground_truth.size());

  for(auto const& z : measurements) {
    
  }
}
