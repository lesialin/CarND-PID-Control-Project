# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

This is the PID control project of self-driving car Term2. 

I implemented PID control algorithm to make the car drive properly in the lane. 

The P/I/D means proportional/derivative/Integral  respectively.

The steering angle is adjusted by:

​	$Steer\ Angle= P*e_{p}+I*e_{I}+D*e_{D}$



## PID Parameter Setting

To selection a proper P/I/D control parameters, I used Twiddle algorithm to optimize these parameters.

At first, I tried the initial PID parameters in {Kp,Ki,Kd} = {0,0,0}, which made car departure from the track easily. 

According to PID control equation, I set smaller value to integral control, lager value to derivative control.

So I manually tuned the initial parameters into {Kp,Ki,Kd} = {0.1, 0.05, 3.0}, and the difference value to adjust the PID control parameters are set to dp[0] = 0.5, dp[1] = 0.01, dp[2] =0.1.

you can find this initialization in main.cpp line 37 `pid.TwiddleInit(0.1, 0.05, 3.0);`

And I set tolorance = 0.01 to be the end criterial to end the twiddle function, which ends as sum (dp) < tolorance.

The hyper-parameters in TwiddleInit, which is in PID.cpp  

```
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

```

After PID parameters optimized by Twiddle function, the PID parameters would be set in PID control initialization. These would be main.cpp 37  to  46 line

```
if (!pid.is_twiddle) {

    //PID parameters from twiddlw function
    std::cout << "tiwddle PID:" << std::endl;
    std::cout << "Kp = " << pid.Kp << std::endl;
    std::cout << "Ki = " << pid.Ki << std::endl;
    std::cout << "Kd = " << pid.Kd << std::endl;
    pid.Init(pid.Kp, pid.Ki, pid.Kd);
  }

```

In the simulation, the optimization parameters are {Kp,Ki,Kd} = {0.05, 0.061, 3.1} from Twiddle algorithm.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

