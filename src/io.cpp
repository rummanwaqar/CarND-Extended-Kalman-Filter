#include "io.h"

using namespace carnd_ekf;

BaseIO::BaseIO(ProcessCb cb)
  : callbackFunc_(cb) {}

void BaseIO::run() {}

void BaseIO::parse_line(const std::string&& line, carnd_ekf::MeasurementPackage& meas_pack,
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

SimIO::SimIO(ProcessCb cb, int port) : BaseIO(cb), port_(port) {
  /*
   * Register event handlers for uWS
   */
  h_.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
     // "42" at the start of the message means there's a websocket message event.
     // The 4 signifies a websocket message
     // The 2 signifies a websocket event
     if (length && length > 2 && data[0] == '4' && data[1] == '2') {
       std::string s = hasData(std::string(data));

       if(s != "") { // data available
         // parse json
         auto j = nlohmann::json::parse(s); std::string event = j[0].get<std::string>();

         if(event == "telemetry") {
           std::string sensor_measurement = j[1]["sensor_measurement"];
           // parse line
           carnd_ekf::MeasurementPackage meas_pack;
           Eigen::VectorXd ground_truth(4);
           parse_line(std::move(sensor_measurement), meas_pack, ground_truth);

           Eigen::VectorXd data = callbackFunc_(meas_pack, ground_truth);

           nlohmann::json msgJson;
           msgJson["estimate_x"] = data(0);
           msgJson["estimate_y"] = data(1);
           msgJson["rmse_x"] =  data(2);
           msgJson["rmse_y"] =  data(3);
           msgJson["rmse_vx"] = data(4);
           msgJson["rmse_vy"] = data(5);
           auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
           ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
         }
       } else {
         std::string msg = "42[\"manual\",{}]";
         ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
       }
     }
  });

  h_.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h_.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
}

void SimIO::run() {
  // listen and wait for connection
  if (h_.listen(port_)) {
    std::cout << "Listening to port " << port_ << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return;
  }
  // endless loop until application exists
  h_.run();
}

std::string SimIO::hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

FileIO::FileIO(ProcessCb cb, std::string& input_name, std::string& output_name) :
  BaseIO(cb), input_file_name_(input_name), output_file_name_(output_name){
  // open file
  in_file_.open(input_file_name_);
  if(!in_file_.good()) {
    throw std::runtime_error("Bad input file");
  }
  out_file_.open(output_file_name_);
}

void FileIO::run() {
  std::string str;
  while(std::getline(in_file_, str)) {
    if(str.size() > 0) {
      // parse line
      carnd_ekf::MeasurementPackage meas_pack;
      Eigen::VectorXd ground_truth(4);
      parse_line(std::move(str), meas_pack, ground_truth);
      // process
      Eigen::VectorXd data = callbackFunc_(meas_pack, ground_truth);
      out_file_ << data(0) << "\t" //px
                << data(1) << "\t" //py
                << data(2) << "\t" //rmse
                << data(3) << "\t"
                << data(4) << "\t" 
                << data(5) << std::endl;
    }
  }
  out_file_.close();
}
