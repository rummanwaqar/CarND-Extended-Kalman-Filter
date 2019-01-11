#ifndef _EKF_IO_H_
#define _EKF_IO_H_

#include <iostream>
#include <sstream>
#include <string>
#include <functional>
#include <uWS/uWS.h>
#include "Eigen/Dense"

#include "json.h"
#include "measurement_package.h"

namespace carnd_ekf {
  class SimIO {
  public:
    SimIO(std::function< Eigen::VectorXd(const std::string&& measurement_string) > cb,
          int port = 4567);
    void run();

  private:
    /*
     * Checks if the SocketIO event has JSON data.
     * If there is data the JSON object in string format will be returned,
     * else the empty string "" will be returned.
     */
    std::string hasData(std::string s);

  private:
    // uWS object
    uWS::Hub h_;

    // connection port
    int port_;

    // processing callback
    // takes a string and returns an eigen vector with px, py rmse(4)
    std::function< Eigen::VectorXd(const std::string&& measurement_string) > callbackFunc_;
  };


} // namespace carnd_ekf

#endif
