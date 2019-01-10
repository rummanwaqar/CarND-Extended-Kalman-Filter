#include "io.h"

using namespace carnd_ekf;

SimIO::SimIO(std::function< Eigen::VectorXd(const std::string&& measurement_string) > cb,
             int port) : port_(port), callbackFunc_(cb) {
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

           Eigen::VectorXd data = callbackFunc_(std::move(sensor_measurement));

           nlohmann::json msgJson;
           msgJson["estimate_x"] = data(0);
           msgJson["estimate_y"] = data(1);
           msgJson["rmse_x"] =  data(2);
           msgJson["rmse_y"] =  data(3);
           msgJson["rmse_vx"] = data(4);
           msgJson["rmse_vy"] = data(5);
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
