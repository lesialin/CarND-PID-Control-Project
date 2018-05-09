#include "PID.h"
#include <cmath>
#include <iostream>
using namespace std;
#define MAX_ERROR  1

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;


}

void PID::UpdateError(double cte) {

  d_error = cte - p_error; // difference error
  p_error = cte; //current error
  i_error += cte;//integral error
  i_error  = i_error > MAX_ERROR ? MAX_ERROR : i_error;
  i_error  = i_error < -MAX_ERROR ? -MAX_ERROR : i_error;
  if (twiddle_state != DONE) {
    if (iter  > n_step) {
      error += cte * cte;
    }
  }

  iter++;
}


double PID::TotalError() {
  double error;
  error = -(Kp * p_error + Ki * i_error + Kd * d_error);
  error  = error > MAX_ERROR ? MAX_ERROR : error;
  error  = error < -MAX_ERROR ? -MAX_ERROR : error;
  return error;
}


void PID::TwiddleInit(double Kp, double Ki, double Kd) {
  
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  dp[0] = 0.05;
  dp[1] = 0.01;
  dp[2] = 0.1;
  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;

  tolorance = 0.01;
  n_step = 100;
  is_twiddle = true;
  iter  = 0;
  best_error = 0.0;
  error = 0.0;
  twiddle_state = IDLE;
  i = 0;


}

void PID::Twiddle() {
  static double current_err = 999.9;
  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;
  //cout << "iter = " << iter << endl;
  if (iter > 2 * n_step) {

    switch (twiddle_state) {

    case DONE:
      cout << "set twiddle done!" << endl;
      is_twiddle = false;
      break;

    case IDLE:
      if (best_error == 0) {
        cout << "error before divede =" << error << endl;
        best_error = error / (2 * n_step);
        cout << "init best error" << endl;
        cout << "best_error =" << best_error << endl;
      }
      if (dp[0] + dp[1] + dp[2] > tolorance) {
        cout << "------twiddle PID param:-------" << endl;
        p[i] += dp[i];
        twiddle_state = INC;
      } else {
        cout << "set twiddle state done!" << endl;
        twiddle_state = DONE;
      }
      break;

    case INC:
      cout << "---------STATE INC----------" << endl;
      cout << "error before divede =" << error << endl;
      current_err = error / (2 * n_step);
      cout << "increse param, get current error" << endl;
      cout << "current error = " << current_err << endl;
      if (current_err < best_error) {
        best_error = current_err;
        dp[i] *= 1.1;
        //cout << "increase dp:" << endl;
        //cout << "dp[0],dp[1],dp[2]=" << dp[0] << "," << dp[1] << "," << dp[2]; << endl;
      } else {
        //cout << "decrease p:" << endl;
        //cout << "p[0],p[1],p[2]=" << p[0] << "," << p[1] << "," << p[2]; << endl;
        p[i] -= 2 * dp[i];
      }
      //set twiddle to DEC
      twiddle_state = DEC;
      break;
    case DEC:
      cout << "---------STATE DEC----------" << endl;
      cout << "error before divede =" << error << endl;
      current_err = error / (2 * n_step);
      cout << "decrease param, get current error" << endl;
      cout << "current error = " << current_err << endl;
      if (current_err < best_error) {
        best_error = current_err;
        dp[i] *= 1.1;
      } else {
        p[i] += dp[i];
        dp[i] *= 0.9;
      }

      twiddle_state = IDLE;

    }//end of switch

    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
    cout << "best error=" << best_error << endl;
    cout << "current error=" << current_err << endl;
    cout << "current dp:" << endl;
    cout << "dp[0],dp[1],dp[2]=" << dp[0] << "," << dp[1] << "," << dp[2] << endl;
    cout << "current p:" << endl;
    cout << "Kp = " << Kp << endl;
    cout << "Ki = " << Ki << endl;
    cout << "Kd = " << Kd << endl;

    error = 0;
    iter = 0;
    i++;
    if (i > 3) {
      i = 0;
    }
  }//if iterate > 2*nstep


}

