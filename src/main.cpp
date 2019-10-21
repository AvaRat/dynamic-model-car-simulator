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
    ros::Rate r(2);
    ros::Publisher pose_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    double dT = 0.001;
    size_t n_cmd = 10000; 
    double torque = 100;
    double steering_angle_1 = 0.2617993878;
    double steering_increment = 0.01;
    double steering_angle_2 = 0.2617993878;
    double steering_angle_3 = 0;
    double steering_tmp = 0;
    int steering_change_1 = 0;
    int steering_change_2 = 0;
    size_t torque_change = 0;

    map<string, double> params;

    fstream param_file;
    param_file.open("/home/marcel/catkin_ws/src/dynamic-model-car-simulator/params.txt", fstream::in);
    if(param_file.is_open())
        cout << "file opening... OK\n";
    string param_name;
    double value;
    while(param_file >> param_name)
    {
        param_file >> value;
        params[param_name] = value;
    }
    
    steering_angle_1 = params["steering_angle_1"];
    steering_angle_2 = params["steering_angle_2"];
    steering_angle_3 = params["steering_angle_3"];
    steering_change_1 = params["steering_change_1"];
    steering_change_2 = params["steering_change_2"];
    dT = params["dT"];
    n_cmd = params["n_cmd"];
    torque = params["torque"];
    torque_change = params["torque_change"];

    param_file.close();

    Model model(dT);
    fstream f;
    f.open("/home/marcel/catkin_ws/src/dynamic-model-car-simulator/data.csv", fstream::out);
    vector<double> cmd_vel;
    vector<double> angle_vec;
    for(size_t i=0; i<torque_change;i++)
        cmd_vel.push_back(torque);

    for(size_t i=0; i<steering_change_1;i++)
        angle_vec.push_back(steering_angle_1);

    f << "x" << "," << "y" << "," << "t" << "," <<  "torque," << "steering_angle" << "," << "long_vel" \
        << "," << "lat_vel" << "," << "yaw_angle" << "," << "yaw_rate," << "slip_angle_f," \
        << "slip_angle_r," << "norm_load_f," << "norm_load_r," << "slip_angle_est_f," << "slip_angle_est_r," <<"lat_for_f, " << "lat_for_r" << endl;
    steering_tmp = steering_angle_1;
    int count = 0;
    cout << steering_change_2 << endl;
    //main loop
    for(size_t n=0; n< n_cmd; n++)
    {   
       
        cmd_vel.push_back(0);
        if(n > steering_change_2 - steering_change_1)
            angle_vec.push_back(steering_angle_3);
        else
        {
            if(fabs(steering_angle_2-steering_tmp) > 1e-10)
            {
                steering_tmp += steering_increment;
                angle_vec.push_back(steering_tmp);
            }else
                angle_vec.push_back(steering_angle_2);
        }
        
        



        auto data = model.get_data();
        f << data["x"]<<","<<data["y"]<<","<<n*dT<<"," << cmd_vel[n]<<"," <<angle_vec[n] << ","<< data["long_vel"]<<","<< data["lat_vel"]<<",";
        f << data["yaw_angle"]<<","<< data["yaw_rate"]<<","<< data["slip_angle_f"]<<"," <<data["slip_angle_r"] << ",";
        f << data["norm_load_f"]<<","<< data["norm_load_r"]<<","<< data["slip_angle_est_f"]<<"," ;
        f << data["slip_angle_est_r"] << "," << data["lat_for_f"] << "," << data["lat_for_r"] << endl;

       // model.publish_pose(&pose_pub);
        model.command(cmd_vel[n], angle_vec[n]);
        if(count++ == 100)
        {
          count = 0;
          model.publish_pose(&pose_pub);
         // r.sleep();
        }
    
    }
    f.close();
     cout << "closing_file\nif_counter: ";
     cout << model.wrong_slip << endl;

    return 0;
}
