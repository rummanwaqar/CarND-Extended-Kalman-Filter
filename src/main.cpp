#include <iostream>
#include <string>
#include <vector>


#include "io.h"
#include "measurement_package.h"
#include "fusion.h"
#include "kalman_filter.h"
#include "tools.h"

std::string FILE_NAME = "../data/obj_pose-laser-radar-synthetic-input.txt";
std::string OUTPUT_FILE = "../data/output.txt";
int PORT = 4567;

Eigen::VectorXd process(const std::string&& measurement_string) {
  std::cout << measurement_string << std::endl;
  Eigen::VectorXd data(6);
  data << 1, 2, 3, 4, 5, 6;
  return data;
}

int main() {
  carnd_ekf::SimIO sim_io(process, PORT);
  sim_io.run();
  
  // load measurements
  // std::vector<carnd_ekf::MeasurementPackage> measurements;
  // std::vector<Eigen::VectorXd> ground_truth;
  // assert(carnd_ekf::read_data_file(FILE_NAME, std::ref(measurements), std::ref(ground_truth)));
  // assert(measurements.size() == ground_truth.size());
  //

  //
  // std::vector<Eigen::VectorXd> estimates;
  // // process all measurements
  // for(auto const& z : measurements) {
  //   fusion.process_measurement(z);
  //   estimates.push_back(fusion.get_state());
  // }
  //
  // Eigen::VectorXd RMSE = carnd_ekf::calculateRMSE(estimates, ground_truth);
  // std::cout << RMSE << std::endl;
  //
  // carnd_ekf::write_output_csv(OUTPUT_FILE, estimates);

}
