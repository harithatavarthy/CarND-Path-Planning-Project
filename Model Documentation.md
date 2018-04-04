# CarND-MPC-Project - MODEL PREDICTIVE CONTROLLER

## Reflection

Objective of the project is to build a Model Predictive controller that can provide steer and throttle commands to the simulator in such a way that the vehcile can reliably drive around the track by predicting where the vehicle will be in the next few time steps. Such ability to predict and control ensures smooth navigation through out track unlike PID controller which has many jerks and breaks. The simulator provides the required 'way points', 'vehicles position in map cooridnates, 'velocity' and 'Orientation angle' for the controller to use as inputs and to arrive at commands to pass back to the simulator.

The solution provided is robust enough to handle an inherent latency of 100 Milli seconds - meaning, there will be approximately 100 milli second latency between arriving at the optimal values for actuation and the application of the actuation itself. The provided solution  also ensures the vehicle stays on track all the time and none of the tires  leave the drivable portion of the track surface even when the vehicle attains speeds of 80 Mile Per Hour. The trick to achieve this is by knowing the magnitude of penalty by which the vehicle should be penalized.




## Project Rubric Discussion Points

### The Model

The kinematic model includes the vehicle's x and y coordinates, orientation angle (psi), and velocity, as well as the cross-track error and psi error (epsi). Actuator outputs are acceleration and delta (steering angle). The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the equations below:

![equations](./eqns.png)

### Timestep Length and Elapsed Duration (N & dt)

At first I tried following the lesson guidelines by setting 'N' with a value of '25' (High) and  'dt' with a value of '0.05'.
Thought it helps me predict the vehicles position for the next 2.5 seconds in future,  the tradeoff is that it gets computationally expensive . I used Mac with i3 processor for this project and realized that solver timed out very quickly. So, i started experimenting with other values and finally settled with a value of '10' for 'N' and '0.1' for 'dt'.  These values ensured that i am not predicting too much or too less.

### Polynomial Fitting and MPC Preprocessing

A polynomial is fitted to waypoints by first converting the simulator provided way points from the Map Coordinate system to Vehicle Coordinate system. This transformation is essential as it simplifies the prociess of fitting a third degree polynomial with an original of (0,0). It will also help us visualize the way points as well as the predicted path. Refer to lines 108 to 113 of source code 'Main.CPP' to see how the transformation is achieved.



`for (int i = 0; i < ptsx.size(); i++) {
double dx = ptsx[i] - px;
double dy = ptsy[i] - py;
waypoints_x[i] = (dx * cospsi - dy * sinpsi);
waypoints_y[i] = (dx * sinpsi + dy * cospsi);
}`




### Model Predictive Control with Latency

With a latency of 100 milli seconds between the acutuation command will not propogate through the system instantly. It will not be of much use to calculate the error with respect to the present state as the vehicle would have already been in a future state by the time the command is passed to the controller. To account for this latency it makes sense to model the latency using a dynamic system where we calculate the vehicles state at the next time step and passing this next state as input to the MPC Solver/Optimizer. The implementation to handle this latency can be found at lines 128 through 139 of source code 'Main.CPP'.


`double px_act = v * 0.1;
double py_act = 0;
double psi_act = - v * steer_value * dt / Lf;
double v_act = v + throttle_value * dt;
double cte_act = cte + v * sin(epsi) * dt;
double epsi_act = epsi + psi_act;
Eigen::VectorXd state(6);
state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;`


Here's the link to the video that shows how the vehicle navigated around the track using Model Predictive Controller that i have build
https://youtu.be/iBmxdjps9gc





