#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include "Eigen/Dense"

#include "io.h"
#include "measurement_package.h"
#include "fusion.h"
#include "kalman_filter.h"
#include "tools.h"

std::string FILE_NAME = "../data/obj_pose-laser-radar-synthetic-input.txt";
std::string OUTPUT_FILE = "../data/output.txt";
int PORT = 4567;

carnd_ekf::Fusion fusion;

void parse_line(const std::string&& line, carnd_ekf::MeasurementPackage& meas_pack,
                Eigen::VectorXd& ground_truth) {
  std::istringstream iss(line);
  long long timestamp;
  std::string sensor_type;
  iss >> sensor_type;
  // parse sensor data
  if(sensor_type.compare("L") == 0) { // lidar
    meas_pack.sensor_type = carnd_ekf::MeasurementPackage::LIDAR;
    meas_pack.raw_measurements = Eigen::VectorXd(2);
    float px, py;
    iss >> px >> py;
    meas_pack.raw_measurements << px, py;
    iss >> timestamp;
    meas_pack.timestamp = timestamp;
  } else if (sensor_type.compare("R") == 0) { // radar
    meas_pack.sensor_type = carnd_ekf::MeasurementPackage::RADAR;
    meas_pack.raw_measurements = Eigen::VectorXd(3);
    float rho, phi, rho_dot;
    iss >> rho >> phi >> rho_dot;
    meas_pack.raw_measurements << rho , phi, rho_dot;
    iss >> timestamp;
    meas_pack.timestamp = timestamp;
  }
  // parse ground_truth
  float x_gt, y_gt, vx_gt, vy_gt;
  iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
  // our state is [x, vx, y, vy]
  ground_truth << x_gt, vx_gt, y_gt, vy_gt;
}


Eigen::VectorXd process(const std::string&& measurement_string) {
  static std::vector<Eigen::VectorXd> estimates;
  static std::vector<Eigen::VectorXd> ground_truths;

  // parse line
  carnd_ekf::MeasurementPackage meas_pack;
  Eigen::VectorXd ground_truth(4);
  parse_line(std::move(measurement_string), meas_pack, ground_truth);

  // process
  fusion.process_measurement(meas_pack);
  Eigen::VectorXd state = fusion.get_state();
  estimates.push_back(state);
  ground_truths.push_back(ground_truth);

  Eigen::VectorXd RMSE = carnd_ekf::calculateRMSE(estimates, ground_truths);

  std::cout << RMSE << std::endl << std::endl;

  Eigen::VectorXd output(6);
  output << state(0), state(2), RMSE(0), RMSE(2), RMSE(1), RMSE(3);
  return output;
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
