#include<iostream>
#include<vector>
#include<fstream>

#include"../include/car_model.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

class Simulator {
private:
  ros::Subscriber cmd_pub;
  ros::Publisher pose_pub;
  ros::Publisher speed_pub;
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;
  tf::StampedTransform transform;
  Model model;
  fstream data_file;
  

public:
  Simulator():model(0) {}
  Simulator(const ros::NodeHandle &n, double dT): nh(n), model(dT)
  {
    ros::Publisher pose_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher speed_pub = nh.advertise<std_msgs::Float64>("speed", 1);
    ros::Subscriber cmd_sub = nh.subscribe("model_control", 1000, &Simulator::cmd_callback, this);
    data_file.open("/home/marcel/catkin_ws/src/dynamic-model-car-simulator/data.csv", fstream::out);
    if(data_file.is_open())
    {
        cout << "OK\n";
        data_file << "x" << "," << "y" << "," << "t" << "," <<  "torque," << "steering_angle" << "," << "long_vel" \
        << "," << "lat_vel" << "," << "yaw_angle" << "," << "yaw_rate," << "slip_angle_f," \
        << "slip_angle_r," << "norm_load_f," << "norm_load_r," << "slip_angle_est_f," << "slip_angle_est_r," <<"lat_for_f, " << "lat_for_r" << endl;
    }else 
    {
      cout << "FAILED\n";
    }
  }

  void cmd_callback(const std_msgs::Float64MultiArray &msg)
  {
    model.command(msg.data[1], msg.data[0]);
    std::map<std::string, double> data;
    model.get_data(data);
    parse_data(data_file, data);
    transform.setOrigin( tf::Vector3(data["x"], data["y"], 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, data["yaw_angle"]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    std_msgs::Float64 speed_msg;
    double abs_speed = (sqrt(pow(data["long_vel"],2) + pow(data["lat_vel"], 2)));
    if(data["long_vel"] > 0)
      speed_msg.data = abs_speed;
    else
      speed_msg.data = -abs_speed;

    speed_pub.publish(speed_msg);
  }

  void parse_data(fstream &file, std::map<std::string, double> data)
  {
    file << data["x"]<<","<<data["y"]<<","<<data["t"]<<"," << data["torque"] <<"," << data["steering_angle"] << ","<< data["long_vel"]<<","<< data["lat_vel"]<<",";
    file << data["yaw_angle"]<<","<< data["yaw_rate"]<<","<< data["slip_angle_f"]<<"," <<data["slip_angle_r"] << ",";
    file << data["norm_load_f"]<<","<< data["norm_load_r"]<<","<< data["slip_angle_est_f"]<<"," ;
    file << data["slip_angle_est_r"] << "," << data["lat_for_f"] << "," << data["lat_for_r"] << endl;
  }
};

