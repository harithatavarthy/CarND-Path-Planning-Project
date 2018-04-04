# CarND-Path-Planning-Project

## Reflection

Objective of the project is to build a path planner for an autonomous car driving on a 3-lane highway. The path planner uses Sensor Data, Map Data and Localization techniques to comprehend the behaviour of the surroundings objects and prepare inputs to the cars controller in such a way that the car maneuvers smoothly and without colliding other vehicles. The path planner starts by reading the sensor data and generate prediction of the current and future states of the vehicular traffic around the autonomous car. Then it relies on  Finite state machines to identify the next set of available trajectories given the current state of the car and produces kinematics for each of these possible trajectories. A cost function will help identify the best trajectory to follow. Finally the controller  smoothens this trajectory with the help of a spline tool.

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





