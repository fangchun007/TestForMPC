# Model Predictive Control (MPC)

This is a test for the project of model predictive control.

Code line 87-92: just simply multiplied the present speed and curvature at this point, then add the result to the cost.

Code line 76-84 is not necessary. They are used for testing and observation.

## The Model

```
    State vector: [x, y, psi, v, cte, epsi]
    
        x: position of vehicle in the x direction
        y: position of vehicle in the y direction
        psi: orientation of vehicle
        v: speed of vehicle
        cte: cross track error
        epsi: orientation error
    
    Actuator: [delta, a]
    
        delta: steering angle [-25°, 25°]
        a: acceleration/decceleration [-1, 1]
    
    Update equations:
    
        x_{t+1}    = x_t + v_t * \cos(psi_t) * dt                   
        y_{t+1}    = y_t + v_t * \sin(psi_t) * dt                    
        psi_{t+1}  = psi_t + v_t * delta_t * dt / Lf                 
        v_{t+1}    = v_t + a_t * dt                               
        cte_{t+1}  = f(x_t) - y_t + v_t * \sin(epsi_t) * dt    
        epsi_{t+1} = psi_t - psides_t + v_t * delta_t * dt / Lf
```

## Optimization / Nonlinear Programming

The nature of Model Predictive Control (MPC) is to reframe the task of following a trajectory as an  optimization/nonlinear programming problem. The solution of the optimization problem is the optimal trajectory. 

With more detail, MPC simulate different actuating inputs and predict corresponding trajectory, then select the trajectory with the minimal cost. Let's assume that the current state [x, y, psi, v, cte, epsi] and the reference trajectory we want to follow or waypoints  are known. MPC optimize actuator [delta, a] in order to minimize the cost of predicted trajectory. Once the lowest cost trajectory is found, we implement the very first actuation. Then we take a new state and use that with the new reference trajectory to calculate a new optimal trajectory. In this sense, MPC is actually a constantly calculating new optimal trajectory method. 

We describe the nonlinear programming problem in one step of MPC as follows. 
```
    Minimize \sum_{t=1}^{N} c_1 * (cte_t - cte_ref)^2 +
                            c_2 * (epsi_t - epsi_ref)^2 +
                            c_3 * (v_t - v_ref)^2 +
                            c_4 * delta_t^2 +
                            c_5 * a_t^2 +
                            c_6 * (cte_{t+1} - cte_t)^2 +
                            c_7 * (epsi_{t+1} - epsi_t)^2 +
                            c_8 * curvature_t * v_t +
                            \cdots
    Subject to -\inf < x_t < +\inf                                             for t \in\{1, 2, \cdots, N\}
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

### Timestep Length N & Elapsed Duration dt

Note that the duration N\*dt over which future predictions are made will determine the length of predictive trajectory (green line in the simulator). It further determines how much future information will be collected and be used to produce a good actuator. This is critical when the vehicle is driving around a curve. After several tries at the same speed of 30m/s, such as (N,dt)=(20,0.05),(20,0.07),(20,0.1),(15,0.07),(15,0.1),(10,0.1),(10,0.15),(8,0.1). e find the duration 1s can contain enough future infomation, even when the vehicle is driving round a curve. 
Thus N determines the number of variables optimized by the MPC. This is also the major driver of computational cost.





