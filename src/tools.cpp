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
    std::stof(tokens[tokens.size()-6].c_str()), // x_gt
    std::stof(tokens[tokens.size()-4].c_str()), // vx_gt
    std::stof(tokens[tokens.size()-5].c_str()), // y_gt
    std::stof(tokens[tokens.size()-3].c_str()); // vy_gt
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

void carnd_ekf::write_output_csv(const std::string& file_name,
                                 const std::vector<Eigen::VectorXd> &estimates) {
  std::cout << "Writing output to: " << file_name << std::endl;
  std::ofstream output_file;
  output_file.open(file_name);
  for(auto const& estimate : estimates) {
    output_file << estimate(0) << "\t" // px
                << estimate(2) << "\t" // py
                << estimate(1) << "\t" // vx
                << estimate(3) << std::endl; //vy
  }
  output_file.close();
}

Eigen::VectorXd carnd_ekf::calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                         const std::vector<Eigen::VectorXd> &ground_truth) {
  Eigen::VectorXd rmse(4); rmse << 0,0,0,0;
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }
  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {
    Eigen::VectorXd residual = estimations[i] - ground_truth[i];
    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  // calculate the mean
  rmse = rmse/estimations.size();
  // calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}
