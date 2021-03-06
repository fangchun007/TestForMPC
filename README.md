# Model Predictive Control (MPC)

## Introduction

In this project, we aim to develop a model predictive controller (MPC) to drive the vehicle around a track in Udacity's [simulator](https://github.com/udacity/self-driving-car-sim/releases), which communicates telemetry and track waypoints via websocket. As a return, MPC produces a steering value and a acceleration/deceleration value and send them back to the simultor to control the vehicle. 

## The Model

The model we used here is a kinematic bicycle model.

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
                            c_6 * (delta_{t+1} - delta_t)^2 +
                            c_7 * (a_{t+1} - a_t)^2 +
                            c_8 * (cte_{t+1} - cte_t)^2 +
                            c_9 * (epsi_{t+1} - epsi_t)^2 +
                            c_10 * curvature_t * v_t +
                            \cdots
    Subject To -\inf < x_t < +\inf                                             for t \in\{1, 2, \cdots, N\}
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

### Polynomial Fitting and MPC Preprocessing

Note that the above model and the return values to the simulator, including the MPC predicted trajectory, the waypoints/reference line, are or can be described in the vehicle's system. There is only one moment that we need to receive way points from the map coordinate system. For convenience, we will tranfer these waypoints from the map/global coordinate system to the vehicle/local coordinate system.
```
    void globalToLocal(vector<double> &ptsx, vector<double> &ptsy, double px, double py,
                       double psi, Eigen::VectorXd &xvals, Eigen::VectorXd &yvals)
    {
      for (size_t i=0; i<ptsx.size(); i++)
      {
        double dx = ptsx[i] - px;
        double dy = ptsy[i] - py;
        xvals[i] = dx * cos(-psi) - dy * sin(-psi);
        yvals[i] = dx * sin(-psi) + dy * cos(-psi);
      }
    }
```
Then we use a third order polynomial to fit a reference trajectory in the vehicle system.

## Parameter Tuning

### Timestep Length N & Elapsed Duration dt

Note that the duration N\*dt over which future predictions are made will determine the length of predictive trajectory (green line in the simulator). It further determines how much future information will be collected and be used to produce a good actuator. This is critical when the vehicle is driving around a curve. After several tries at the same speed of 30 m/s, such as (N,dt) = (20,0.05), (20,0.07),(20,0.1), (15,0.07), (15,0.1), (10,0.1), (10,0.15), (8,0.1), we decide to choose (N,dt) = (10,0.1) as the candidate in our coming experiments. First, it looks like the duration time 2s is unnecessarily long, which will lower down the computation efficiency, increase the cost, and actually lower down the accuracy (note we only implement the very first actuate). Second, when the duration time is less than 1s, we start to concern whether we can obtain enough front infomation to make a good decision. We also found if dt is 0.05, the vehicle adjust its orientation too frequent to obtain a stable drive even along an almost straight line. We didn't choose dt=0.15 although it worked well, because we expect some troubles when we try to reach a higher speed.

### Latency

From the file *main.cpp*, one can find the following command.
```
    this_thread::sleep_for(chrono::milliseconds(100));
```
Namely, the simulator will recieve actuator values with a latency of at least 100 milliseconds. This is used to mimic the real latency of a real car. In this project, we use the elapse time of function *h.onMessage* during the last step as an estimated latency of the present step. 

To do that, we use *time_pre* to record the time moment when the previous round of *h.onMessage()* begin to work. 
```
    time_pre = std::chrono::system_clock::now();
```
And use *latency_pre* to save the elapse time of finishing the function *h.onMessage()* during the previous step.

```
std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - time_pre;
double latency_pre = elapsed_seconds.count();
```
To handle with some unexpected high latency value, we ignore the latency values higher than 0.25 s.

### Tuning The Cost Function

#### c1, c2, c3, c4, c5, c6, c7, c8, c9

In the begining, we won't consider the affection of curvature of the reference line. Namely, set c10 = 0. Let's start with the tuning of parameters c1, c2, ..., c9 (corresponding to x, y, psi, v, cte, epsi respectively). Intuitively, the bigger ci is, the higher influence for the i-th component. After several tries, we find that the part of cost caused by steering angle, corresponding to c4, is critical to a stable drive. It is reasonable since in a stable driving (duration time N*\dt) one don't need too much steering. For similar reason, we give a high rate to the gap between sequential steering angles. To decrease oscillations, we also consider and give high rates to the gap between sequential cross track error and epsi. In order to obtain a good acceleration/deceleration when run along a curve, we don't plan to punish too much on c5 and c7. In the end, we obtained an OK but possibly not the best parameters.
```
    (c1, c2, c3, c4, c5, c6, c7, c8, c9) = (10, 2, 1, 1000, 1, 200, 1, 100, 200)
```
#### c10

With the above setting, the vehicle can run at a speed of around 60 mph. To reach a higher speed, one method is to decrease N from 10 to 9 or 8. The idea is to increase the accuracy of optimizated solution. In this way, it is possible that increasing the *Numeric max_cpu_time* can be also helpful. 
```
    options += "Numeric max_cpu_time          3\n"; 
```
In this project, we consider the curvature of the reference trajectory instead. By observation, we find that the green line (predicted trajectory) and yellow line (reference trajectory) agree well when the vehicle is driving along a straight line. However, their differences become big when the vehicle is turning. We are going introduce the product of the curvature of the reference trajectory and the present speed to the cost function, so that we can control the speed of vehicle when it is turning. 

```
    // Add the affection of curvature
    for (size_t t=0; t<N-1; t++)
    {
      AD<double> numerator = CppAD::abs(2 * coeffs[2] + 6 * coeffs[3] * vars[x_start + t]);
      AD<double> denominator = CppAD::pow(1 + CppAD::pow(coeffs[1] + 2 * coeffs[2] * vars[x_start + t] + 3 * coeffs[3] * vars[x_start + t] * vars[x_start + t], 2), 3/2);
      fg[0] += 1000 * numerator/denominator * vars[v_start + t];
    }
```
**Local expression of curvature**
Suppose the curve is $y = f(x)$. The the curvature at point $x$ is:
```
    \kappa = \frac{\|f''(x)\|}{(1+f'^2(x))^{3/2}}
```
If we print out the curvatures, one can find the values are around 0.001 to 0.04. Since the cost when c10=0 is around 50, it is reasonable to set c10 to 1000. Of course, one can adjust this parameter further in order to obtain much better result.  







