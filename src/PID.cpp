#include "PID.h"
#include <limits>
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  //Initializes PID coefficients (and errors, if needed)

   Kp_ = Kp;   //tau_p
   Ki_ = Ki;   //tau_i
   Kd_ = Kd;   //tau_d

   p_error_ = 0.0;   //cte
   i_error_ = 0.0;   //sum(cte)
   d_error_ = 0.0;   //cte_diff

   stepNumber_ = 1;
   bestError_ = std::numeric_limits<double>::max(); //Initialize to highest double value

   //twiddle updates every 200 steps
   numSteps_ = 50;

   //Store coefficients
   p_[0] = Kp_*0.1;
   p_[1] = Ki_*0.1;
   p_[2] = Kd_*0.1;

   //Current index of coefficients
   currentIndex_ = 0;
  
   addRequired_ = subRequired_ = false;
}

void PID::UpdateError(double cte) {
   //Updates PID errors based on cte.

   //Set the intial p_error to the cte, which can be used as the prev_cte in the next cycle
   if(stepNumber_ == 1) {
     p_error_ = cte;
   }

   //PID equations
   d_error_ = cte - p_error_;
   p_error_ = cte;
   i_error_ += cte;

  /**
    * Twiddle - Hyperparameter Tuning
    */

  //Get total error based on cross track error
  totalError_ += cte*cte;

  //Run the twiddle algorithm every 'n' evaluation steps
  //if(step_num % numSteps == 0) {
  if((stepNumber_ < 5000) && (stepNumber_%numSteps_ == 0)) {
    //if the current error is a new best, update
    if(totalError_ < bestError_) {
      bestError_ = totalError_;
      p_[currentIndex_] *= 1.1;

      addRequired_ = subRequired_ = false;
    }

    if(!addRequired_ && !subRequired_) {
      //First iteration after start of cycle, add elements
      Twiddler(currentIndex_, p_[currentIndex_]);
      addRequired_ = true;
    } else if(addRequired_ && !subRequired_) {
      //Second iteration after cycle
      //No best error found,
      Twiddler(currentIndex_, -2*p_[currentIndex_]);
      subRequired_ = true;
    } else {
      //Third iteration
      //No best error found after two attempts, time to try something new
      Twiddler(currentIndex_, p_[currentIndex_]);
      p_[currentIndex_] *= 0.9;
      addRequired_ = subRequired_ = false;

      //Cycle through the 3 hyperparameters
      currentIndex_ = (currentIndex_ + 1) % 3;
    }
    //Reset total error at end of cycle
    totalError_ = 0;

    //Debugging prompts
    std::cout << "Kp is: " << Kp_ << " Ki is: " << Ki_ << " Kd is: " << Kd_ << "\n\n";
  }

   stepNumber_++;
}

double PID::TotalError() {
  //Calculate and return the total error
  return -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;
}

//Twiddles variables based on index of hyperparameter and value determined above.
void PID::Twiddler(int index, double value) {
  switch(index) {
    case 0:
      Kp_ += value;
      break;
    case 1:
      Ki_ += value;
      break;
    case 2:
      Kd_ += value;
      break;
    default:
      break;
  }
}