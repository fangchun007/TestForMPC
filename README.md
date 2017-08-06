# TestForMPC

This is a test for the project of model predictive control.

Code line 87-92: just simply multiplied the present speed and curvature at this point, then add the result to the cost.

Code line 76-84 is not necessary. They are used for testing and observation.


1. Optimization/Nonlinear programming

The nature of Model Predictive Control (MPC) is to reframe the task of following a trajectory as an  optimization/nonlinear programming problem. The solution of the optimization problem is the optimal trajectory. In other words, MPC simulate different actuating inputs and predict corresponding trajectory, then select the trajectory with the minimal cost. 

With more detail, let's assume that the current state [x, y, psi, v, cte, epsi] and the reference trajectory we want to follow are known. MPC optimize actuator [delta, acceleration] in order to minimize the cost of predicted trajectory. Once the lowest cost trajectory is found, we implement the very first actuation. Then we take a new state and use that with the new reference trajectory to calculate a new optimal trajectory. In that sense, MPC is actually a constantly calculating new optimal trajectory method. 

The following is the model of MPC in one step. 

    minimize \sum_{t=0}^{N} c_1 * (cte_t - cte_ref)^2 +
                            c_2 * (epsi_t - epsi_ref)^2 +
                            c_3 * (v_t - v_ref)^2 +
                            c_4 * delta_t^2 +
                            c_5 * a_t^2 +
                            c_6 * (cte_{t+1} - cte_t)^2 +
                            c_7 * (epsi_{t+1} - epsi_t)^2 +
                            c_8 * curvature_t * v_t +
                            \cdots
               

2. Parameter Tuning

