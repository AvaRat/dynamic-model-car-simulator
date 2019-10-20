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
    ros::Rate r(10);
    ros::Publisher pose_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    double dT = 0.001;
    size_t n_cmd = 10000; 
    double torque = 100;
    double steering_angle = 0.2617993878;
    size_t torque_n_cmd = 0;

    map<string, double> params;

    fstream param_file;
    param_file.open("/home/marcel/catkin_ws/src/dynamic-model-car-simulator/params.txt", fstream::in);
    if(param_file.is_open())
        cout << "is_open\n";
    string param_name;
    double value;
    while(param_file >> param_name)
    {
        param_file >> value;
        params[param_name] = value;
    }

    steering_angle = params["steering_angle"];
    dT = params["dT"];
    n_cmd = params["n_cmd"];
    torque = params["torque"];
    torque_n_cmd = params["torque_n_cmd"];
    param_file.close();

    Model model(dT);
    fstream f;
    f.open("/home/marcel/catkin_ws/src/dynamic-model-car-simulator/data.csv", fstream::out);
    vector<double> cmd_vel;
    vector<double> angle;
    for(size_t i=0; i<torque_n_cmd;i++)
        cmd_vel.push_back(torque);

    f << "x" << "," << "y" << "," << "t" << "," <<  "torque," << "steering_angle" << "," << "long_vel" \
        << "," << "lat_vel" << "," << "yaw_angle" << "," << "yaw_rate," << "slip_angle_f," \
        << "slip_angle_r," << "norm_load_f," << "norm_load_r," << "slip_angle_est_f," << "slip_angle_est_r," <<"lat_for_f, " << "lat_for_r" << endl;

    int count = 0;
    for(size_t n=0; n< n_cmd; n++)
    {   
       
        cmd_vel.push_back(0);
        angle.push_back(steering_angle);
        auto data = model.get_data();
        f << data["x"]<<","<<data["y"]<<","<<n*dT<<"," << cmd_vel[n]<<"," <<angle[n] << ","<< data["long_vel"]<<","<< data["lat_vel"]<<",";
        f << data["yaw_angle"]<<","<< data["yaw_rate"]<<","<< data["slip_angle_f"]<<"," <<data["slip_angle_r"] << ",";
        f << data["norm_load_f"]<<","<< data["norm_load_r"]<<","<< data["slip_angle_est_f"]<<"," ;
        f << data["slip_angle_est_r"] << "," << data["lat_for_f"] << "," << data["lat_for_r"] << endl;

       // model.publish_pose(&pose_pub);
        model.command(cmd_vel[n], angle[n]);
        if(count++ == 100)
        {
          count = 0;
          model.publish_pose(&pose_pub);
        //  r.sleep();
        }
    
    }
    f.close();
//     cout << "closing_file\n";
     cout << model.wrong_slip << endl;

    return 0;
}
