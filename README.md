# Model Predictive Control (MPC)

This is a test for the project of model predictive control.

Code line 87-92: just simply multiplied the present speed and curvature at this point, then add the result to the cost.

Code line 76-84 is not necessary. They are used for testing and observation.

## The Model

```
    state vector: [x, y, psi, v, cte, epsi]
        x: position of vehicle in the x direction
        y: position of vehicle in the y direction
        psi: orientation of vehicle
        v: speed of vehicle
        cte: cross track error
        epsi: orientation error
    
    actuator: [delta, a]
        delta: steering angle [-25°, 25°]
        a: acceleration/decceleration [-1, 1]
```
## The Model

### state vector: [x, y, psi, v, cte, epsi]
* x: position of vehicle in the x direction
        * y: position of vehicle in the y direction
        * psi: orientation of vehicle
        * v: speed of vehicle
        * cte: cross track error
        * epsi: orientation error
    
### actuator: [delta, a]
 * delta: steering angle [-25°, 25°]
 * a: acceleration/decceleration [-1, 1]


## Optimization/Nonlinear Programming

The nature of Model Predictive Control (MPC) is to reframe the task of following a trajectory as an  optimization/nonlinear programming problem. The solution of the optimization problem is the optimal trajectory. In other words, MPC simulate different actuating inputs and predict corresponding trajectory, then select the trajectory with the minimal cost. 

With more detail, let's assume that the current state [x, y, psi, v, cte, epsi] and the reference trajectory we want to follow are known. MPC optimize actuator [delta, acceleration] in order to minimize the cost of predicted trajectory. Once the lowest cost trajectory is found, we implement the very first actuation. Then we take a new state and use that with the new reference trajectory to calculate a new optimal trajectory. In that sense, MPC is actually a constantly calculating new optimal trajectory method. 

The following is the model of MPC in one step. 
```
    minimize \sum_{t=1}^{N} c_1 * (cte_t - cte_ref)^2 +
                            c_2 * (epsi_t - epsi_ref)^2 +
                            c_3 * (v_t - v_ref)^2 +
                            c_4 * delta_t^2 +
                            c_5 * a_t^2 +
                            c_6 * (cte_{t+1} - cte_t)^2 +
                            c_7 * (epsi_{t+1} - epsi_t)^2 +
                            c_8 * curvature_t * v_t +
                            \cdots
    subject to -\inf < x_t < +\inf                                             for t \in\{1, 2, \cdots, N\}
               -\inf < y_t < +\inf                                             for t \in\{1, 2, \cdots, N\}
               -\inf < psi_t < +\inf                                           for t \in\{1, 2, \cdots, N\}
               -\inf < v_t < +\inf                                             for t \in\{1, 2, \cdots, N\}
               -\inf < cte_t < +\inf                                           for t \in\{1, 2, \cdots, N\}
               -\inf < epsi_t < +\inf                                          for t \in\{1, 2, \cdots, N\}
               -0.436332 <= delta_t <= 0.436332                                for t \in\{1, 2, \codts, N-1\}
               -1 <= a_t <= 1                                                  for t \in\{1, 2, \codts, N-1\}
               x_1    = x1
               y_1    = y1
               psi_1  = psi1
               v_1    = v1
               cte_1  = cte1
               epsi_1 = epsi1
               x_{t+1} - (x_t + v_t * \cos(psi_t) * dt) = 0                    for t \in\{1, 2, \codts, N-1\}
               y_{t+1} - (y_t + v_t * \sin(psi_t) * dt) = 0                    for t \in\{1, 2, \codts, N-1\}
               psi_{t+1} - (psi_t + v_t * delta_t * dt / Lf)                   for t \in\{1, 2, \codts, N-1\}
               v_{t+1} - (v_t + a_t * dt) = 0                                  for t \in\{1, 2, \codts, N-1\}
               cte_{t+1} - (f(x_t) - y_t + v_t * \sin(epsi_t) * dt) = 0        for t \in\{1, 2, \codts, N-1\}
               epsi_{t+1} - (psi_t - psides_t + v_t * delta_t * dt / Lf) = 0   for t \in\{1, 2, \codts, N-1\}
               
               
               
```                            
## Parameter Tuning

