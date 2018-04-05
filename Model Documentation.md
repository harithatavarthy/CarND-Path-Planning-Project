# CarND-Path-Planning-Project

## Reflection

Objective of the project is to build a path planner for an autonomous car driving on a 3-lane highway. The path planner uses Sensor Data, Map Data and Localization techniques to comprehend the behaviour of the surroundings objects and prepare inputs to the cars controller in such a way that the car maneuvers smoothly and without colliding other vehicles. The path planner starts by reading the sensor data and generate prediction of the current and future states of the vehicular traffic around the autonomous car. Then it relies on  Finite state machines to identify the next set of available states/trajectories given the current state of the car and produces kinematics for each of these possible trajectories. A cost function will help identify the best trajectory to follow. Finally the planning process ends by fitting a lane through the start and end points and smoothens the trajectory with the help of a spline tool.

The car was able to successfully complete four laps around the highway without exceeding  accelaration/Jerk limits. The car stayed in the center of the lane except when changing lanes. The car was able to sense the traffic around it in a timely fashion and adjust the speed whenever required. Also the car chose the the best lane to follow based on the average lane speed.

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
  
  ## Predictions using Sensor Fusion Data
  
  It is important for the path planning process to have informaiton on ever changing dynamic data of vehicular traffic around the autonomous car in order to come up with the best trajectory at each time step. We will read the sensor data and create a prediction set of current and future positions of the vehicular traffic. The code for this can be found between lane # 287 and 318 of main.cpp
  
           `vector<Vehicle> traffic;
            map<int, vector<Vehicle>> predictions;
            
            
            for(int i=0;i<sensor_fusion.size();i++)
            {
               
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
                check_car_s += ((double)prev_size*.02*check_speed);
                double check_car_d = sensor_fusion[i][6];
                //double check_car_future_s = check_car_s + (1 * check_speed);
                int traffic_id = sensor_fusion[i][0];
                int traffic_lane_now = -1;
                for(int j=0; j<3 ; j++)
                {
                    if(check_car_d <= (2+(4*j)+2) && check_car_d > (2+(4*j)-2))
                    {
                        traffic_lane_now = j;
                    }
                        
                }

                
                Vehicle otherv = Vehicle(traffic_lane_now,check_car_s,check_car_d,check_speed,0,"KL");
                
                traffic.push_back(otherv);
                predictions[traffic_id] = traffic;
                
            }`
           
## Finite State machines to transition between states

Path planner uses Finite State machines to transition from one state to another (keep lane, prepare lane change, lane change). Given the limited number of states used for path planning, FSM is easy to maintain.  The definition of the possible state transition for the FSM can be found between lane # 73 and 100 of vehicle.cpp

           `vector<string> Vehicle::successor_states()
           {
               /*
                Provides the possible next states given the current state for the FSM
                , with the exception that lane changes happen
                instantaneously, so LCL and LCR can only transition back to KL.
                */

               vector<string> states;
               states.push_back("KL");
               string state = this->state;
               if(state.compare("KL") == 0) {
                   states.push_back("PLCL");
                   states.push_back("PLCR");
               } else if (state.compare("PLCR") == 0) {
                   if (lane != lanes_available - 1) {
                       states.push_back("PLCR");
                       states.push_back("LCR");
                   }
               } else if (state.compare("PLCL") == 0) {
                   if (lane != 0) {
                       states.push_back("PLCL");
                       states.push_back("LCL");
                   }
               }

               return states;
           }`

          
## Kinematics

For each of the possible state(s) defined by the FSM, the path planner obtains the kinematics required to execute the state. For example, if the current state of the vehicle is 'KL - Keep Lane', the next possible set of states are 
 a. Continue to Keep Lane
 b. Prepare Lane change Left
 c. Prepare Lane change Right

For each of these three possible states, the path planner creates trajectories by calling the function `get_kinematics`. The function returns the the new position , new velocity and accelration required to form and execute the trajectory for that state. The code for `get_kinematics` function can be found between lane # 121 and 160 of vehicle.cpp. 

           `vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
               /*
                Gets next timestep kinematics (position, velocity, acceleration)
                for a given lane. Tries to choose the maximum velocity and acceleration,
                given other vehicle positions and accel/velocity constraints.
                */
               float max_velocity_accel_limit = this->max_acceleration + this->v;
               if(max_velocity_accel_limit > 49.5)
               {
                   max_velocity_accel_limit = 49.5;
               }

               float new_position;
               float new_velocity;
               float new_accel;
               Vehicle vehicle_ahead;
               Vehicle vehicle_behind;

               if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

                   if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
                       new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer

                   }
                   else {
                       float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 *                                                  (this->a);
                       new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);


                   }
               } else {
                   new_velocity = min(max_velocity_accel_limit, this->target_speed);

               }

               new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
               new_position = this->s + new_velocity + new_accel / 2.0;
               return{new_position, new_velocity, new_accel};

           }`
 
## Cost Function

The only cost function used for this project is 'Efficiency' related . For each of the possible trajectories, the cost function returns the cost of executing that trajectory. The path planner choses the one with the lowest cost. In this implementation of cost function, the average speed of the lane the trajectory follows is computed by looking at the vehicular traffic in that lane with a set horizon. The one with the lesser speed will have higher cost. The code for this cost function can be found between lane # 35 to 56 of cost.cpp.

           `float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane,float position) {
               /*
                All non ego vehicles have different speeds, so to get the speed limit for a lane will look at a horizon of say 100                        meters
                and obtain the average velocity of all traffic in that lane with in that horizon.
                */
               int count = 0;
               int avgVelocity = -1;
               for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
                   int key = it->first;
                   Vehicle vehicle = it->second[it->first];
                   if (vehicle.lane == lane && key != -1 && vehicle.s > position && (vehicle.s - position) < 100) {
                       avgVelocity += vehicle.v;
                       count++;
                   }
                   if(count > 0)
                   {
                       return avgVelocity/count;
                   }
               }
               //Found no vehicle in the lane
               return -1.0;
           }`






