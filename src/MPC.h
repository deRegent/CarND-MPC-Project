#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:

  size_t timestep_length;
  double timestep_duration;

  double cte_cost;
  double epsi_cost;
  double v_cost;

  double delta_cost;
  double a_cost;

  double delta_gap_cost;
  double a_gap_cost;

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
