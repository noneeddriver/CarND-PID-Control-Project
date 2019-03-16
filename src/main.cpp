// Autor: Pengmian Yan
// Last modifified on: March 16, 2019
// Version: 1.0

#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::cout;
using std::endl;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;


  /**
   * TODO: Initialize the pid variable.
   */
  PID pid_longitudinal;
  PID pid_lateral;

  pid_longitudinal.Init(0.1, 0.000, 3.0); // Kp_, Ki_, Kd_

  bool twiddle_mode = false;
  bool first_cycle = true;
  double best_error = 100000000000.0;
  double p [3] = {0.2, 0.003, 5.0}; // initial parameter
  unsigned int p_size = sizeof(p) / sizeof(*p);
  double dp [3] = {1.0, 1.0, 1.0};
  unsigned int iterator_p = 0;
  unsigned int n = 0;
  unsigned int max_n = 100;
  double tolerance = 0.02;
  bool try_positive_side = true;
  bool p_updated = false;

  if (twiddle_mode) {
    pid_lateral.Init(p[0], p[1], p[2]);
  }
  else {
    pid_lateral.Init(0.2, 0.003, 5.0); // Kp_, Ki_, Kd_
  }

  h.onMessage([&pid_longitudinal, &pid_lateral, &twiddle_mode, &first_cycle, &p, &dp, &p_size,
               &iterator_p, &n, &max_n, &best_error, &tolerance, &try_positive_side, &p_updated]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double speed_desired = 40.0 - 3.0 * abs(angle);
          speed_desired = (speed_desired < 10.0) ? 10.0: speed_desired;
          double throttle;

          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */


//          pid_lateral.Init(0.184057558862, 3.09047640185, 0.0);

          pid_longitudinal.UpdateError(speed - speed_desired);

          pid_lateral.UpdateError(cte);
          pid_lateral.TotalError();
//          cout << "total error: " << pid_lateral.TotalError() << endl;

//          pid_lateral.twiddle(0.2);
          // DEBUG
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
//                    << std::endl;
          steer_value = pid_lateral.GetControValue();
          throttle = pid_longitudinal.GetControValue();
          steer_value = (steer_value > 1.0) ? 1.0 : steer_value;
          steer_value = (steer_value < -1.0) ? -1.0 : steer_value;


          n += 1;
          // Twiddle the best parameter for lateral PID controler
          if (twiddle_mode && dp[0] + dp[1] + dp[2] > tolerance && n > max_n) {
            std::cout << "p: " << "Kp = "<< p[0] << "   Ki = " << p[1] << "   Kd = " << p[2] << std::endl;
            std::cout << "dp: " << "dKp = "<< dp[0] << "   dKi = " << dp[1] << "   dKd = " << dp[2] << std::endl;
            double error = pid_lateral.total_error;
            std::cout << "error = " << error << "    best error = " << best_error << std::endl;
            if (first_cycle) {
              best_error = error;
              first_cycle = false;
            }
            else {
              if (!p_updated) {
                p[iterator_p] += dp[iterator_p];
                p_updated = true;
              }
              else {
                if (error < best_error) {
                  best_error = error;
                  dp[iterator_p] *= 1.1;
                  iterator_p += 1;
                  iterator_p = iterator_p % p_size;
                  p_updated = false;
                  try_positive_side = true;
                }
                else {
                  if (try_positive_side) {
                    try_positive_side = false;
                    p[iterator_p] -= 2.0 * dp[iterator_p];
                  }
                  else {
                    p[iterator_p] += dp[iterator_p];
                    dp[iterator_p] *= 0.9;
                    iterator_p += 1;
                    iterator_p = iterator_p % p_size;
                    p_updated = false;
                    try_positive_side = true;
                  }
                }
              }
            }
            // reset the car
            pid_longitudinal.Init(0.1, 0.000, 3.0); // Kp_, Ki_, Kd_
            pid_lateral.Init(p[0], p[1], p[2]);
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            std::cout << "car reseted at " << n << " steps." << std::endl;
            n = 0;
          }
          else if (twiddle_mode && dp[0] + dp[1] + dp[2] <= tolerance) {
            cout << "best parameter: pd = " << p[0] << "  pi = " << p[1] << "  pd = " << p[2] << endl;
            return 0;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
