# TestForMPC

This is a test for the project of model predictive control.

Code line 87-92: just simply multiplied the present speed and curvature at this point, then add the result to the cost.

Code line 76-84 is not necessary. They are used for testing and observation.


1. Optimization/Nonlinear programming

The nature of Model Predictive Control (MPC) is to reframe the task of following a trajectory as an  optimization/nonlinear programming problem. The solution of the optimization problem is the optimal trajectory. In other words, MPC simulate different actuating inputs and predict corresponding trajectory, then select the trajectory with the minimal cost. 

2. Parameter Tuning

