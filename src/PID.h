#ifndef PID_H
#define PID_H

class PID {
 public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;
  unsigned int i;
  unsigned int iter;
  unsigned int n_step;
  double dp[3];
  double p[3];
  double tolorance;
  double best_error;
  double error;
  bool is_twiddle;

  enum {IDLE, INC, DEC, DONE} twiddle_state;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  void TwiddleInit(double Kp, double Ki, double Kd);
  void Twiddle();
};


#endif /* PID_H */
