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
  ros::Publisher speed_pu;
  Model model;
  fstream data_file;
  

}