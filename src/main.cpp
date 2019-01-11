#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "io.h"
#include "fusion.h"
#include "tools.h"

int PORT = 4567;

carnd_ekf::Fusion fusion;

Eigen::VectorXd process(const carnd_ekf::MeasurementPackage& meas_pack,
                        const Eigen::VectorXd& ground_truth) {
  static std::vector<Eigen::VectorXd> estimates;
  static std::vector<Eigen::VectorXd> ground_truths;

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

int main(int argc, char** argv) {
  carnd_ekf::BaseIO* io_interface;

  std::string input_file_name;
  std::string output_file_name;

  // if input file provided then file mode otherwise sim mode
  if(argc > 1) {
    input_file_name = std::string(argv[1]);
  }
  if(argc > 2) {
    output_file_name = std::string(argv[2]);
  } else {
    output_file_name = "../data/output.txt";
  }

  if(input_file_name != "") {
    // file mode
    std::cout << "Reading from :" << input_file_name << std::endl;
    std::cout << "Writing to: " << output_file_name << std::endl;
    io_interface = new carnd_ekf::FileIO(process, input_file_name,
                                         output_file_name);
  } else {
    // sim mode
    std::cout << "Connecting to simulator" << std::endl;
    io_interface = new carnd_ekf::SimIO(process, PORT);
  }

  io_interface->run();
  delete io_interface;
  return 0;
}
