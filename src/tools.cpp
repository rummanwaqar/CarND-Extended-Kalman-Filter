#include "tools.h"

void carnd_ekf::split_by_tab(const std::string&& line, std::vector<std::string>& tokens) {
  std::istringstream ss(line);
  std::string token;
  while(std::getline(ss, token, '\t')) {
    tokens.push_back(token);
  }
}

Eigen::VectorXd carnd_ekf::get_ground_truth(std::vector<std::string>& tokens) {
  Eigen::VectorXd gt_values(4);
  gt_values <<
    std::stof(tokens[tokens.size()-4].c_str()), // x_gt
    std::stof(tokens[tokens.size()-3].c_str()), // y_gt
    std::stof(tokens[tokens.size()-2].c_str()), // vx_gt
    std::stof(tokens[tokens.size()-1].c_str()); // vy_gt
  return gt_values;
}

bool carnd_ekf::read_data_file(const std::string& file_name,
  std::vector<carnd_ekf::MeasurementPackage>& measurements,
  std::vector<Eigen::VectorXd>& ground_truth) {
  // open file
  std::ifstream in_file(file_name);
  if(!in_file.good()) {
    return false;
  }
  // read file line by line
  std::string str;
  int index = 0;
  while(std::getline(in_file, str)) {
    if(str.size() > 0) {
      // split line by tabs
      std::vector<std::string> tokens;
      split_by_tab(std::move(str), tokens);
      // append measurements/ground-truth
      if(tokens[0] == "R" || tokens[0] == "L") {
        measurements.push_back(carnd_ekf::MeasurementPackage(tokens));
        ground_truth.push_back(carnd_ekf::get_ground_truth(tokens));
      } else {
        std::cout << "Invalid input on line " << index+1 << std::endl;
      }
      index++;
    }
  }
  std::cout << "Read " << index << " lines from " << file_name << std::endl;
  // close file and return
  in_file.close();
  return true;
}
