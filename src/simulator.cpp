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
  ros::Subscriber cmd_sub;
  ros::Publisher pose_pub;
  ros::Publisher speed_pub;
  tf::TransformBroadcaster br;
  tf::StampedTransform transform;
  Model model;
  fstream data_file;
  

public:
  Simulator():model(0) {}
  ~Simulator()
  {
    data_file.close();
    cout << "closing data file in destructor\n";
  }
  Simulator(ros::Publisher &pose_pub, ros::Publisher &speed_pub, double dT)
  :model(dT), pose_pub(pose_pub), speed_pub(speed_pub)
  {
    ROS_INFO("opening data file...  ");
    data_file.open("/home/marcel/Documents/Article_RTOS_2020/ros_ws/src/dynamic-model-car-simulator/data.csv", fstream::out);
    if(data_file.is_open())
    {
        data_file << "x" << "," << "y" << "," << "t" << "," <<  "torque," << "steering_angle" << "," << "long_vel" \
        << "," << "lat_vel" << "," << "yaw_angle" << "," << "yaw_rate," << "slip_angle_f," \
        << "slip_angle_r," << "norm_load_f," << "norm_load_r," << "slip_angle_est_f," << "slip_angle_est_r," <<"lat_for_f, " << "lat_for_r" << endl;
        ROS_INFO("OK\n");
    }else 
    {
      ROS_INFO("FAILED");
    }
  }

  void cmd_callback(const std_msgs::Float64MultiArray &msg)
  {
    //ROS_INFO("cmd");
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
  void publish_pose()
  {
    geometry_msgs::Pose pose;
   // pose.orientation.
  }

  void set_cmd_subscriber(ros::Subscriber &sub)
  {
    cmd_sub = sub;
  }
};

