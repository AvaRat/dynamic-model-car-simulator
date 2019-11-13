#include<iostream>
#include<vector>
#include<map>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>



class Model {
private:
	visualization_msgs::Marker marker;
	double long_pos;    //longitudinal position
	double lat_pos;     //lateral position
	double long_vel;    //longitudinal velocity
	double lat_vel;     //lateral velocity
	double long_acc;		//y_acceleration
	double lat_acc;			//x_acceleration
	
	double yaw_angle;
	double yaw_rate;
	double mass;
	double cf;  //cornering stiffnes of front tires
	double cr; //cornering stiffness of rear tires
	double u;
	double a;   //distance from center of gravity to front axle
	double b;   //distance from center of gravity to rear axle
	double h;   //height of center of gravity
	double r; //wheel radius
	double i; //moment of inertia
	double dT; //time t1-t0
	double g; //gravitational acceleration
	double long_for_f; //longitudinal force of front tires
	double long_for_r; //longitudinal force of rear tires
	double lat_for_f; //lateral tire force on front tires
	double lat_for_r; //lateral tire force on front and rear tires
	double slip_angle_f; //slip angle of front tires
	double slip_angle_r; //slip angle of rear tires
	double norm_load_f; //normal tire load on front tires
	double norm_load_r; //normal tire load on rear tires
	double slip_angle_est_f; //slip angle of front tires (estimation)
	double slip_angle_est_r; //slip angle of front tires (estimation)
	double max_torque;



public:


	Model(double dT, double initial_speed);
	void publish_pose(ros::Publisher *pub);
	void execute_command(double torque, double steering_angle);
	std::vector<double> get_position(); // returns long_pos, lat_pos and yaw_angle
	void get_data(std::map<std::string, double> &data);
	double get_max_torque();
	void set_friction_coef(double);
	int wrong_slip = 0; //zmienna pomocnicza
};
