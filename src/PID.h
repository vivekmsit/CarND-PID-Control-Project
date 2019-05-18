#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void Twiddler(int index, double value);

 private:
  /**
   * PID Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;

  /**
   * PID Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;


  //Current step
  int stepNumber_;

  //Total steps per cycle, 'n' steps
  int numSteps_;

  /**
   * Twiddle variables
   */
  double bestError_;
  double totalError_;
  int currentIndex_;
  bool addRequired_;
  bool subRequired_;
  double p_[3];
};

#endif  // PID_H