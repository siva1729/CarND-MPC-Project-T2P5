# CarND-Controls-MPC - Self-Driving Car Nanodegree Program

## Project Overview:
--------------------
This project involves using the Model Predictive Control (MPC), to drive a simulated car around a virtual track provided the waypoints from the track (using a simulator). Implementation is in C++, and involves calculating the actuator values (Steering angle and acceleration/throttle) for the car to drive/stay on track.

## Implementation 
------------------

### Model:
Line-fitting based on the given waypoints and compare/evaluate to that of current state of prediction.
Set the time of Horizon (N) over which the state is predicted for time step (tuning parameter)
Then calculate the actuator values (accerleration and steering angle) based on state and line fitting. In relaity there is difference in the setting of actuator values and its application becasue of delay (100 ms). And this would cause the state to change and this is accounted for for the delay time (100 ms).
Using the solver calculate the Steering angle/throttle values for the Car.
Cost function is set/calculated for different conditions that needs to be met to safely drive the car around the track and at certain speed (cross-track-error, steering-angle-error, velocity error, etc.)
To make the car stay on track, involves tuning the time step, and the weight values for the cost calculations.


### Timestep Length and Elapsed Duration (N & dt):
The timestep length (N) would determine the horizon time to predict into the future state of the car
but at the same time imply more processing/calculations along with the elapsed time duration (dt)
between time steps. So "N" has been chosen to start with a value of 20 seconds and dt = 10 msec,
this caused the calculations (especially) the optimizer to return values little late and not in time to simulator causing the car to off-track. The values has been tuned along with other params and finally settled on N=10 and dt = 100msec which gave a reasonable performance.

### Polynomial Fitting and MPC Preprocessing
Using the waypoints provided by the simulator, the track ahead is estimated/fitted to a third degree polynomial using the 6-points provided along the track. Before the curve fitting the waypoints have been transformed to vehicle corordinate reference system such that vehicle location is at origin and the vehicle is 
along the horizontal axis to the track (approximation). Then to fit the polynomial the polyfot equation
provided at following link is used.
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716

Using the polynomial the initial cross-track-error and Steering-angle-error are caluclated with the assumption that current car'slocation is at origin (0,0).

### Model Predictive Control with Latency
The simulator has eastimate of 100 msec latency for application of actuator values, so this condition is handled by estimating the predicted state variables of the car for the 100 msec latency using the initial 
state conditions and using the steering angle/acceleration values provided by the simulator. Then the
solver (MPC) is called to calulate the actuator values based on the delayed state vector.

### Results:
The project is implemented to account for the latency (of 100 ms) in actuator application. The simulated car has been made drive around the track without going over board and with velocity of up to 125 mph. 

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.



