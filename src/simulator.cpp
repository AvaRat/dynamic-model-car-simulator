#include<iostream>
#include<vector>
#include<fstream>

#include"../include/car_model.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include"std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

class Simulator {
private:
	ros::Subscriber cmd_sub;
	
	ros::Publisher pose_pub;
	ros::Publisher speed_pub;
	ros::Publisher acc_pub;
  tf::TransformBroadcaster br;
	tf::StampedTransform transform;

  	Model model;
  	double dT;
  	double actual_time = 0; //seconds
  	double actual_error = 0; //from track in meters
  	double track_progress = 0;
  	double torque = 0;  //last torque command
  	double steering_angle = 0;  //last steering_angle command
  	double max_torque = 0;  //max torque that will be passed to
		bool paused = true;			//variable to pause simulator and not execute new commands/ however still publishing tf and pose

  	fstream data_file;

public:
	Simulator():model(0, 0) {}  //DO NOT USE default constructor
	~Simulator()
	{
	  data_file.close();
	  cout << "closing data file in destructor\n";
	}
	Simulator(ros::Publisher &pose_pub, ros::Publisher &speed_pub, ros::Publisher &acc_pub_, double DT, double initial_speed)
	:model(DT, initial_speed), dT(DT), pose_pub(pose_pub), speed_pub(speed_pub), acc_pub(acc_pub_)
	{
	  ROS_INFO("opening data file...  ");
	  max_torque = model.get_max_torque();
	
	  data_file.open("/home/marcel/catkin_ws/src/dynamic-model-car-simulator/data.csv", fstream::out);
	  if(data_file.is_open())
	  {
		  parse_header();
		  ROS_INFO("OK\n");
	  }else
	  {
		ROS_INFO("FAILED");
	  }
	}

	void next_time_step()
	{
		  std::map<string, double> data;
			if(paused == false)
      	model.execute_command(torque, steering_angle);
			model.get_data(data);
			get_simulation_data(data);
	  	transform.setOrigin( tf::Vector3(data["x"], data["y"], 0.0));
	  	tf::Quaternion q;
	  	q.setRPY(0, 0, data["yaw_angle"]);
	  	transform.setRotation(q);
	  	//publish tf speed and pose to ROS
	  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
	  	publish_pose(data);
	  	publish_speed(data);
			publish_acceleration(data);

			//parse all data to file
			if(!paused)
			{
				parse_data(data);
				//time progress
				actual_time+=dT; 
			}
	}
	void pause_callback(std_msgs::Bool msg)
	{
		paused = msg.data;
	}
	void set_cmd_subscriber(ros::Subscriber &sub)
	{
	  cmd_sub = sub;
	}

	void cmd_callback(const std_msgs::Float64MultiArray &msg)
	{
	 	//ROS_INFO("torque: %lf", msg.data[1]);
	  //ROS_INFO("steering_angle: %lf", msg.data[0]);
	  torque = msg.data[1];
	  steering_angle = msg.data[0];
	  if(torque > max_torque) torque = max_torque;
	  else if(torque < -max_torque) torque = -max_torque;
	}
	void parse_data(std::map<std::string, double> &data)
	{
	  data_file << data["x"]<<","<<data["y"]<<","<<data["t"]<<"," << data["torque"] << "," << data["steering_angle"] << ",";
		data_file << data["long_acc"] << ","<< data["long_vel"]<<","<< data["lat_acc"] << "," << data["lat_vel"]<<",";
	  data_file << data["yaw_angle"]<<","<< data["yaw_rate"]<<","<< data["slip_angle_f"]<<"," <<data["slip_angle_r"] << ",";
	  data_file << data["norm_load_f"]<<","<< data["norm_load_r"]<<","<< data["slip_angle_est_f"]<<"," ;
	  data_file << data["slip_angle_est_r"] << "," << data["lat_for_f"] << "," << data["lat_for_r"] << ",";
	  data_file << data["track_progress"] << "," << data["error"] << endl;
	}
	void track_progress_callback(const std_msgs::Float64 msg)
	{
	  track_progress = msg.data;
	}
	void error_callback(const std_msgs::Float64 msg)
	{
	  actual_error = msg.data;
	}
	void publish_speed(std::map<string, double> &data)
	{
	  std_msgs::Float64 speed_msg;
	  double abs_speed = (sqrt(pow(data["long_vel"],2) + pow(data["lat_vel"], 2)));
	  if(data["long_vel"] > 0)
		speed_msg.data = abs_speed;
	  else
		speed_msg.data = -abs_speed;

	  speed_pub.publish(speed_msg);
	}
	void publish_pose(map<string, double> &data)
	{
	  geometry_msgs::PoseStamped pose;
	  pose.header.frame_id = "map";
	  pose.header.stamp = ros::Time::now();
	  pose.pose.position.x = data["x"];
	  pose.pose.position.y = data["y"];
	  pose.pose.position.z = 0;
	  pose.pose.orientation = to_quaternion(data["yaw_angle"], 0, 0);

	  pose_pub.publish(pose);
	}
	void publish_acceleration(map<string, double> &data)
	{
		std_msgs::Float64 acc_msg;
		double abs_acc = (sqrt(pow(data["long_acc"],2) + pow(data["lat_acc"], 2)));
		if(data["long_acc"] > 0)
		acc_msg.data = abs_acc;
	  else
		acc_msg.data = -abs_acc;

	  acc_pub.publish(acc_msg);
	}

	geometry_msgs::Quaternion to_quaternion(double yaw, double pitch, double roll)
	{
	  geometry_msgs::Quaternion q;
	  double cy = cos(yaw * 0.5);
	  double sy = sin(yaw * 0.5);
	  double cp = cos(pitch * 0.5);
	  double sp = sin(pitch * 0.5);
	  double cr = cos(roll * 0.5);
	  double sr = sin(roll * 0.5);
	  q.w = cy * cp * cr + sy * sp * sr;
	  q.x = cy * cp * sr - sy * sp * cr;
	  q.y = sy * cp * sr + cy * sp * cr;
	  q.z = sy * cp * cr - cy * sp * sr;
	  return q;
	}
	void get_simulation_data(map<string, double> &data)
	{
		data["track_progress"] = track_progress;
	  data["error"] = actual_error;
	  data["t"] = actual_time;
	  data["steering_angle"] = steering_angle;
	  data["torque"] = torque;
	}
	void parse_header()
	{
	  data_file << "x" << "," << "y" << "," << "t" << "," <<  "torque," << "steering_angle" << "," << "long_acc," << "long_vel" \
		<< "," << "lat_acc," << "lat_vel" << "," << "yaw_angle" << "," << "yaw_rate," << "slip_angle_f," \
		<< "slip_angle_r," << "norm_load_f," << "norm_load_r," << "slip_angle_est_f," << "slip_angle_est_r,"\
		<<"lat_for_f, " << "lat_for_r," <<"track_progress," << "error" << endl;
	}
};