#include <iostream>
#include <string>
#include <vector>
#include <uWS/uWS.h>

#include "json.h"
#include "measurement_package.h"
#include "fusion.h"
#include "kalman_filter.h"
#include "tools.h"

std::string FILE_NAME = "../data/obj_pose-laser-radar-synthetic-input.txt";
std::string OUTPUT_FILE = "../data/output.txt";
int PORT = 4567;

using json = nlohmann::json;

/*
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 */
std::string hasData(std::string s) {
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

int main() {
  // uWS object
  uWS::Hub h;

  // initialize filter
  carnd_ekf::Fusion fusion;

  /*
   * Register event handlers for uWS
   */
  h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
     // "42" at the start of the message means there's a websocket message event.
     // The 4 signifies a websocket message
     // The 2 signifies a websocket event
     if (length && length > 2 && data[0] == '4' && data[1] == '2') {
       std::string s = hasData(std::string(data));


       if(s != "") {
         auto j = json::parse(s);
         std::string event = j[0].get<std::string>();

         if(event == "telemetry") {
           std::string sensor_measurement = j[1]["sensor_measurement"];
           std::cout << sensor_measurement << std::endl;

           json msgJson;
           msgJson["estimate_x"] = 0;
           msgJson["estimate_y"] = 0;
           msgJson["rmse_x"] =  0;
           msgJson["rmse_y"] =  0;
           msgJson["rmse_vx"] = 0;
           msgJson["rmse_vy"] = 0;
           auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
           // std::cout << msg << std::endl;
           ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
         }
       } else {
         std::string msg = "42[\"manual\",{}]";
         ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
       }
     }
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  // listen and wait for connection
  if (h.listen(PORT)) {
    std::cout << "Listening to port " << PORT << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  // endless loop until application exists
  h.run();

  return 0;

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
