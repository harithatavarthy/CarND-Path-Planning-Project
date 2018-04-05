# CarND-Path-Planning-Project

## Reflection

Objective of the project is to build a path planner for an autonomous car driving on a 3-lane highway. The path planner uses Sensor Data, Map Data and Localization techniques to comprehend the behaviour of the surroundings objects and prepare inputs to the cars controller in such a way that the car maneuvers smoothly and without colliding other vehicles. The path planner starts by reading the sensor data and generate prediction of the current and future states of the vehicular traffic around the autonomous car. Then it relies on  Finite state machines to identify the next set of available trajectories given the current state of the car and produces kinematics for each of these possible trajectories. A cost function will help identify the best trajectory to follow. Finally the planning process ends by fitting a lane through the start and end points and smoothens the trajectory with the help of a spline tool.

## Avoiding Max Acceleration and Jerk

Avoiding sudden changes in acceleration and jerks is key for a smooth and comfortable ride. This is achieved by obtaining the target velocity supplied by the behavior planners FSM (finite state machine) and verifying it for maximum allowable change in acceleration ( 10 meters per second square) and jerk (10 meters per second cube). If the target velocity when compared to vehicle current reference velocity is beyond this maximum allowable limit, the vehicles reference velocity is adjusted in order to contain the sudden jumps in acceleration and jerks. The code for this can be found between lane # 330 and 339 of main.cpp


           `const int MAX_ACCEL= 10; // m/s/s
            const double accel = (MAX_ACCEL) * 0.02 * 0.8; // Limit acceleration within acceptable range
            if (ref_vel < next_state.v - accel) // Accelerate if under target speed
            {
                ref_vel += accel;
            }
            else if (ref_vel > next_state.v + accel)// Brake if above target
            {
                ref_vel -= accel;
            }`
  
  Once the new reference velocity is set, the next step is to create a set of equally spaced way points for the vehicle to traverse. This is achieved by using the target longitudinal displacement (s Value), target lane (d Value), target velocity ( adjusted reference velocity) and combine it with the previous way points that are not yet traversed by the vehicle and interopolate a smooth lane by using a spline tool. Code for this can be found between lane # 348 and 476 of main.cpp.
  
  
           











