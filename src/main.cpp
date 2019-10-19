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

    f << "x" << "," << "y" << "," << "t" << "," <<  "torque" << "," << "long_vel" \
        << "," << "lat_vel" << "," << "yaw_angle" << "," << "yaw_rate," << "slip_angle_f," \
        << "norm_load_f," << "norm_load_r," << "slip_angle_est_f," << "slip_angle_est_r" << endl;
    for(size_t n=0; n< n_cmd; n++)
    {   
       
        cmd.push_back(torque);
        auto data = model.get_data();
        f << data["x"]<<","<<data["y"]<<","<<n*dT<<"," << cmd[n]<<","<< data["long_vel"]<<","<< data["lat_vel"]<<",";
        f << data["yaw_angle"]<<","<< data["yaw_rate"]<<","<< data["slip_angle_f"]<<"," ;
        f << data["norm_load_f"]<<","<< data["norm_load_r"]<<","<< data["slip_angle_est_f"]<<"," ;
        f << data["slip_angle_est_r"]<<endl;

        model.publish_pose(&pose_pub);
        model.command(cmd[n], 0);
      //  r.sleep();
    }
    f.close();
    return 0;
}
