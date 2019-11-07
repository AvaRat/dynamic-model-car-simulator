#include<utility>
#include<fstream>
#include<vector>
#include "../include/dynamic_tree.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"


double dT= 0;
double cmd_change_duration= 1;  //seconds

struct Cmd{
    double torque;
    double angle;
    double duration;
};

void parse_description(fstream &file)
{
    file << "x" << "," << "y" << "," << "t" << "," <<  "torque," << "steering_angle" << "," << "long_vel" \
        << "," << "lat_vel" << "," << "yaw_angle" << "," << "yaw_rate," << "slip_angle_f," \
        << "slip_angle_r," << "norm_load_f," << "norm_load_r," << "slip_angle_est_f," << "slip_angle_est_r," <<"lat_for_f, " << "lat_for_r" << endl;
}

void parse_data(fstream &file, map<string, double> data)
{
    file << data["x"]<<","<<data["y"]<<","<<data["t"]<<"," << data["torque"] <<"," << data["steering_angle"] << ","<< data["long_vel"]<<","<< data["lat_vel"]<<",";
    file << data["yaw_angle"]<<","<< data["yaw_rate"]<<","<< data["slip_angle_f"]<<"," <<data["slip_angle_r"] << ",";
    file << data["norm_load_f"]<<","<< data["norm_load_r"]<<","<< data["slip_angle_est_f"]<<"," ;
    file << data["slip_angle_est_r"] << "," << data["lat_for_f"] << "," << data["lat_for_r"] << endl;
}

vector<Cmd> read_file(fstream &file)
{
    double torque, angle, duration;
    Cmd cmd;
    vector<Cmd> cmd_vec;
    string param_name;
    file >> param_name; //skip first word
    file >> dT;  // get dT
    file >> param_name;
    file >> cmd_change_duration;
    file >> param_name >>param_name >> param_name;  //ignore first line
    while(file >> torque >> angle >> duration)
    {
        cmd.torque =torque;
        cmd.angle = angle;
        cmd.duration = duration;
        cmd_vec.push_back(cmd);
    }

    return cmd_vec;
}



using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator");
    ros::NodeHandle n;
    ros::Rate r(50);
    ros::Publisher pose_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   
    fstream data_file;
    size_t change_n_cmd = 0;
    vector<Cmd> commands;
    fstream param_file;
    param_file.open("/home/marcel/catkin_ws/src/dynamic-model-car-simulator/params.txt");
    cout << "opening params.txt ...";
    commands = read_file(param_file);
    commands.push_back({0, 0, 0.01});
    change_n_cmd = cmd_change_duration/dT;
    if(param_file.is_open())
        cout << "OK\n";
    else 
    {
      cout << "failed\n";
      return -1;
    }
    param_file.close();
    Model model(dT, 0);

    cout << "opening data file...";
    
    data_file.open("/home/marcel/catkin_ws/src/dynamic-model-car-simulator/data.csv", fstream::out);
    if(data_file.is_open())
        cout << "OK\n";
    else 
    {
      cout << "FAILED\n";
      return -1;
    }
    map<string, double> data;
    model.get_data(data);
    parse_description(data_file);
    //model.command(0,0);
    parse_data(data_file, data);

    //main loop of commands
    for(auto it=commands.begin();it!=commands.end(); it++)
    {
        size_t n_cmd = (size_t)(*it).duration/dT;
        for(size_t n=0; n<n_cmd; n++)
        {
            model.command((*it).torque, (*it).angle);
            model.get_data(data);
            parse_data(data_file, data);
            model.publish_pose(&pose_pub);
            // r.sleep();
        }
        //command gradual change
        double angle_increment = ((*it).angle - (*(it+1)).angle) / change_n_cmd;
        double torque_increment = ((*it).torque - (*(it+1)).torque) / change_n_cmd;
        if(it != commands.end()-1)
        {
            for(size_t i=0; i<change_n_cmd; i++)
            {
               // cout <<"ok\n";
                model.command((*it).torque-(torque_increment*i), (*it).angle-(angle_increment*i));
                model.get_data(data);
                parse_data(data_file, data);
                model.publish_pose(&pose_pub);
                // r.sleep();
            }
        }
    }
    cout << "closing data file ... ";
    data_file.close();
    if(!data_file.is_open())
        cout << "OK\n";
    return 0;
}