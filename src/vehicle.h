#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:
    
    map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};
    
    struct collider{
        
        bool collision ; // is there a collision?
        int  time; // time collision happens
        
    };
    
    int lane;
    
    double s;
    
    double v;
    
    double a;
    
    double d;
    
    double next_s;

    int lanes_available = 3;
    
    string state;
    
    float max_acceleration;
    
    int preferred_buffer = 6; // impacts "keep lane" behavior.
    
    float target_speed = 49.5;

    
    /**
     * Constructor
     */
    Vehicle();
    //Vehicle(int lane, double s, double v, double a, double d, double next_s, string state="CS");
    Vehicle(int lane, double s, double d, double v, double a, string state="CS");
    
    /**
     * Destructor
     */
    virtual ~Vehicle();
    
    vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);
    
    vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);
    
    vector<string> successor_states();
    
    vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);
    
    bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
    
    bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
    
    vector<Vehicle> constant_speed_trajectory();
    
    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);
    
    vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
    
    vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
    
    float position_at(int t);
    
};

#endif
