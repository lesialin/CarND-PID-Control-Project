#include "PID.h"
#include <cmath>
#include <iostream>
using namespace std;

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
  if (i_error > 1) {
    i_error = 1;
  }
  if (i_error < -1) {
    i_error = -1;
  }
}



double PID::TotalError() {
  double error;
  error = -(Kp * p_error + Ki * i_error + Kd * d_error);
  error  = error > 1 ? 1 : error;
  error  = error < -1 ? -1 : error;

  return error;
}