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
    /*
     * Constructor
     * @param cb callback for processing function
     */
    BaseIO(ProcessCb cb);

    /*
     * Destructor
     */
    virtual ~BaseIO() = default;

    /*
     * Run the io processor
     */
    virtual void run();

  protected:
    /*
     * parse a string into measurement package and ground truth vector
     * @param line - input string
     * @param meas_pack - parsed measurement package to return
     * @param ground_truth - parsed ground truth to return
     */
    void parse_line(const std::string&& line, carnd_ekf::MeasurementPackage& meas_pack,
                    Eigen::VectorXd& ground_truth);

    // processing callback
    ProcessCb callbackFunc_;
  };

  /*
   * Interface to simulator
   */
  class SimIO : public BaseIO {
  public:
    /*
     * Constructor
     * Creates uWebSocket object and defines all event handlers
     * @param cb callback for processing function
     * @param port - port number for simulator uWebSocket
     */
    SimIO(ProcessCb cb,
          int port = 4567);

    /*
     * Destructor
     */
    ~SimIO() = default;

    /*
     * Initializes connection to simulator and blocks it until simulator is closed.
     * event handling for simulator
     */
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

  /*
   * Interface to file based IO
   */
  class FileIO : public BaseIO {
  public:
    /*
     * Constructor
     * Initializes file IO
     * @param cb callback for processing function
     * @param input_name - input file for data
     * @param output_name - output file to write results
     */
    FileIO(ProcessCb cb, std::string& input_name, std::string& output_name);

    /*
     * Destructor
     */
    ~FileIO() = default;

    /*
     * Reads input file line by line and processes it
     */
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
