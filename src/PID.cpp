#include "PID.h"

#include <limits>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  p_error_ = i_error_ = d_error_ = 0;
  bestError_ = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
  d_error_ = p_error_ - cte;
  p_error_ = cte;
  i_error_ += cte;
  // TODO - Update coefficients here
}

double PID::TotalError() {
  double totalError = -Kp_*p_error_ - Ki_*i_error_ - Kd_*d_error_;
  return totalError;
}