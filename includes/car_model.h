#include<iostream>


class Model {
    private:
    double long_pos;    //longitudinal position
    double lat_pos;     //lateral position
    double long_vel;    //longitudinal velocity
    double lat_vel;     //lateral velocity
    double yaw_angle;   
    double yaw_rate;
    double mass;
    double cf;  //cornering stiffnes
    double a;   //distance from center of gravity to front axle
    double b;   //distance from center of gravity to rear axle
    double h;   //height of center of gravity


    public:
    Model();
    void command(double speed, double steering_angle);
};

