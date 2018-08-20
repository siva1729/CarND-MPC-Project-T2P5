#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

/*** All the constants and tuning parameters for MPC  ***/

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
#define LF 2.67


// Number of steps during time horizon
#define NUM_TIME_STEPS 10
// Time step duration in sec
#define DELTA_TIME 0.1

#define DELAY_TIME 0.1

// Reference Velocity to deal with stopping
#define V_REF 125
// Cost function WEIGHTs
#define CTE_WEIGHT       3000
#define EPSI_WEIGHT      4000
#define V_WEIGHT         1.0
#define DELTA_WEIGHT     5
#define A_WEIGHT         5
#define DELTADIFF_WEIGHT 200
#define ADIFF_WEIGHT     10
// Set Lower/Upper limits for variables.
#define DELTA_LIMIT 0.436332 // 25 Degrees in Radians
#define A_LIMIT 1.0
#define S_LIMIT 1.0e19
using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
