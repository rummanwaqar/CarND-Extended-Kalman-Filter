#ifndef _EKF_IO_H_
#define _EKF_IO_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <functional>
#include <uWS/uWS.h>
#include "Eigen/Dense"

#include "json.h"
#include "measurement_package.h"

namespace carnd_ekf {
  // callback function definition
  // takes measurement and ground truth and returns an eigen vector with px, py rmse(4)
  typedef std::function< Eigen::VectorXd(
    const MeasurementPackage& meas_pack, const Eigen::VectorXd& ground_truth) > ProcessCb;

  class BaseIO {
  public:
    BaseIO(ProcessCb cb);

    virtual ~BaseIO() = default;
    virtual void run();

  protected:
    void parse_line(const std::string&& line, carnd_ekf::MeasurementPackage& meas_pack,
                    Eigen::VectorXd& ground_truth);

    // processing callback
    // takes a string and returns an eigen vector with px, py rmse(4)
    ProcessCb callbackFunc_;
  };

  class SimIO : public BaseIO {
  public:
    SimIO(ProcessCb cb,
          int port = 4567);
    ~SimIO() = default;
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
  };

  class FileIO : public BaseIO {
  public:
    FileIO(ProcessCb cb, std::string& input_name, std::string& output_name);
    ~FileIO() = default;
    void run();

  private:
    // input and output files
    std::string input_file_name_;
    std::string output_file_name_;

    // input / output file streams
    std::ifstream in_file_;
    std::ofstream out_file_;
  };

} // namespace carnd_ekf

#endif
