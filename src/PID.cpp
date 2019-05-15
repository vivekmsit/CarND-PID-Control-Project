#include "PID.h"

#include <limits>
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  p_ = {Kp_,Ki_,Kd_};
  dp_ = {1,1,1};
  dpSumLimit_ = 0.001;
  currentIndex_ = 0;
  checkNumber_ = 0;
  p_error_ = i_error_ = d_error_ = 0;
  bestError_ = std::numeric_limits<double>::max();
  currentStep_ = 0;
  stepComplete_ = false;
  calibrationDone_ = false;
  incr_p_with_dp_done_ = false;
  post_not_best_err_done_ = false;
  iterationComplete_ = false;
}

void PID::UpdateError(double cte) {
  d_error_ = p_error_ - cte;
  p_error_ = cte;
  i_error_ += cte;
  double dpSum = dp_[0] + dp_[1] + dp_[2];
  
  std::cout<<"cte is: " << cte << ", d_error is: " << d_error_ << ", i_error is: " << i_error_ << std::endl;
  
  if (currentStep_ == 0) {
    p_[currentIndex_] += dp_[currentIndex_];
    currentStep_++;
    return;
  }
  
  if ((dpSum > dpSumLimit_ && !calibrationDone_) || (currentStep_ < 500)) {
    std::cout<<"dpSum is: " << dpSum << ", currentStep is: "<<currentStep_<<std::endl;
    if (checkNumber_ == 0) {
      if (cte < bestError_) {
        std::cout<<"checkNumber_ 0, if case"<<std::endl;
        bestError_ = cte;
        dp_[currentIndex_] *= 1.1;
      } else {
        std::cout<<"checkNumber_ 0, else case"<<std::endl;
        p_[currentIndex_] -= 2 * dp_[currentIndex_];
        checkNumber_ = !checkNumber_;
      }
    } else {
      if (cte < bestError_) {
        std::cout<<"checkNumber_ 1, if case"<<std::endl;
        bestError_ = cte;
        dp_[currentIndex_] *= 1.1;
      } else {
        std::cout<<"checkNumber_ 1, else case"<<std::endl;
        p_[currentIndex_] += dp_[currentIndex_];
        dp_[currentIndex_] *= 0.9;
      }
      checkNumber_ = !checkNumber_;
    }
    
    // End of iteration
    if (currentIndex_ == 2) {
      std::cout<<"End of iteration, currentIndex_ is 2"<<std::endl;
      currentIndex_ = 0;
      dpSum = dp_[0] + dp_[1] + dp_[2];
      if (dpSum < dpSumLimit_) {
        // stop updating coefficients
        std::cout<<"stopping updating coefficients"<<std::endl;
        calibrationDone_ = true;
      } else {
        p_[currentIndex_] += dp_[currentIndex_];
      }
    } else {
      std::cout<<"increment of index to "<< currentIndex_+1<<std::endl;
      currentIndex_++;
      p_[currentIndex_] += dp_[currentIndex_];
    }
  } else {
    std::cout<<"Calibration already completed"<<std::endl;
  }
  Kp_ = p_[0];
  Ki_ = p_[1];
  Kd_ = p_[2];
  currentStep_++;
}

double PID::TotalError() {
  double totalError = -Kp_*p_error_ - Ki_*i_error_ - Kd_*d_error_;
  return totalError;
}