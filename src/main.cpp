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


    //simulator stuff
    Path path;
    path.read_from_file("/home/marcel/Documents/Article_RTOS_2020/dynamic-model-car-simulator/symulator.txt");
 //   auto closests = path.get_n_closest(0.87, 6.3001, 3);
    Model model;
    Dynamic_tree tree(&model, &path);
    tree.create_children({10.0,-5.0}, {0.78539, -0.78539});
/*
    double torque, steering_angle;
    while(ros::ok())
    {
        cout << "\ntorque: ";
        cin >> torque;
        cout << "\nsteering_angle: ";
        cin >> steering_angle;

        model.command(torque, steering_angle);
        model.publish_pose(&pose_pub);

        auto vec = model.get_position();
        
        cout << "(x,y) = " << "(" << vec[0] << ", " << vec[1] << ") "<< endl << "theta: " << vec[2] << endl;  
        r.sleep();
    }
*/
    size_t n_cmd = 100;

    fstream f;
    f.open("data.csv", fstream::out);
    double t = 5;
    vector<double> cmd;
    cmd.push_back(10);

    f << "x" << "," << "y" << "," << "yaw" << "," << "torque" << endl;
    for(size_t n=0; n< n_cmd; n++)
    {   
        cmd.push_back(0);
        model.command(cmd[n], 0);
        auto vec = model.get_position();
        f << vec[0] << "," << vec[1] << "," << vec[2] << "," << t << endl;
        model.publish_pose(&pose_pub);
     //   r.sleep();
    }
    return 0;
}