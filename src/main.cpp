#include <iostream>
#include <string>
#include <vector>

#include "measurement_package.h"
#include "fusion.h"
#include "kalman_filter.h"
#include "tools.h"

std::string FILE_NAME = "../data/obj_pose-laser-radar-synthetic-input.txt";

int main() {
  // load measurements
  std::vector<carnd_ekf::MeasurementPackage> measurements;
  std::vector<Eigen::VectorXd> ground_truth;
  assert(carnd_ekf::read_data_file(FILE_NAME, std::ref(measurements), std::ref(ground_truth)));
  assert(measurements.size() == ground_truth.size());

  // initialize filter
  carnd_ekf::Fusion fusion;

  // process all measurements
  for(auto const& z : measurements) {
    if(z.sensor_type == carnd_ekf::MeasurementPackage::LIDAR) {
      fusion.process_measurement(z);
    }
  }
}
