#include "PID.h"
#include <math.h>
#include <iostream>

using std::cout;
using std::endl;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  total_error = 0.0;
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double error) {
  /**
   * TODO: Update PID errors based on cte.
   */
  i_error += error;
  d_error = error - p_error;
  p_error = error;
//  cout << "error: " << p_error << " " << i_error << " " << d_error  << endl;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  total_error += pow(p_error,2.0);
  return total_error;  // TODO: Add your total error calc here!
}

double PID::GetControValue() {
  /**
   * TODO: Calculate and return the total error
   */
  double control_value;
  control_value = -Kp * p_error - Ki * i_error * Kd * d_error;
  return control_value;  // TODO: Add your total error calc here!
}

//Parameter twiddle(double tol) {
//  Parameter p[3] = {0.0, 0.0, 0.0};
//
//
//}
