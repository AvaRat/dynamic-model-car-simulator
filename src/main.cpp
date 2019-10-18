#include<utility>
#include "../include/simulator/dynamic_tree.h"

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
    auto closests = path.get_n_closest(0.87, 6.3001, 3);
    Model model;
    Dynamic_tree tree(&model, &path);
    tree.create_children({10.0,-5.0}, {0.78539, -0.78539});
    while(ros::ok())
    {
        model.command(0, 0);
        model.publish_pose(&pose_pub);
        r.sleep();
    }
    return 0;
}