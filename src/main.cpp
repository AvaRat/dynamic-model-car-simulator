#include<utility>
#include<fstream>
#include<vector>
#include "../include/dynamic_tree.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>


using namespace std;
int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "simulator");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher pose_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    double dT = 0.001;
    size_t n_cmd = 1000;
    double torque = 100;

    Model model(dT);
    fstream f;
    f.open("data.csv", fstream::out);

    vector<double> cmd;

  //  cmd.push_back(torque);

    f << "x" << "," << "y" << "," << "yaw" << "," << "t" << "," <<  "torque" << endl;
    for(size_t n=0; n< n_cmd; n++)
    {   
        if(n >= n_cmd/2)
            torque = -torque;
        cmd.push_back(torque);
        model.command(cmd[n], 0);
        auto vec = model.get_position();
        f << vec[0] << "," << vec[1] << "," << vec[2] << "," << dT*n << "," << cmd[n] << endl;
        model.publish_pose(&pose_pub);
      //  r.sleep();
    }
    f.close();
    return 0;
}
