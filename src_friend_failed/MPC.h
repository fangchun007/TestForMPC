#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class FG_eval; // declare the FG_eval class in advance.

class MPC {
public:
  friend class FG_eval;
public:
  size_t N = 10;
  double dt = 0.1;
  double Lf = 2.67;
  double ref_v = 50;
  size_t x_start = 0;
  size_t y_start = x_start + N;
  size_t psi_start = y_start + N;
  size_t v_start = psi_start + N;
  size_t cte_start = v_start + N;
  size_t epsi_start = cte_start + N;
  size_t delta_start = epsi_start + N;
  size_t a_start = delta_start + N - 1;
  /*
   * Constructor
   */
  MPC(size_t N, double dt, double Lf);
  /*
   * Destructor
   */
  virtual ~MPC();
  
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
};

#endif /* MPC_H */
