#include"../include/simulator/car_model.h"

Model::Model()
{
    long_pos = 1;
    lat_pos = 5;
    yaw_angle = 0.57;


 //   visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/path";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "path";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = 0;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.05;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
}

void Model::command(double speed, double steering_angle)
{
    /*
    std::cout << "command called\n give new coordinates";
    std::cout << "\nx_pos: \t";
    std::cin >> long_pos;
    std::cout << "\ny_pos: \t";
    std::cin >> lat_pos;
    std::cout << "\nangle: \t";
    std::cin >> yaw_angle; 
    std::cout << std::endl;
    */
}

/**
 * @retval vector<double> = {x_pos, y_pos, yaw}
 */
std::vector<double> Model::get_position()
{
    std::vector<double> vec = {long_pos, lat_pos, yaw_angle};
    return vec;
}

void Model::publish_pose(ros::Publisher *pub)
{
    marker.pose.position.x = long_pos;
    marker.pose.position.y = lat_pos;
    double w, x, y, z;
    double cy = cos(yaw_angle * 0.5);
    double sy = sin(yaw_angle * 0.5);
    double cp = cos(0 * 0.5);
    double sp = sin(0 * 0.5);
    double cr = cos(0 * 0.5);
    double sr = sin(0 * 0.5);

 //   Quaternion q;
    w = cy * cp * cr + sy * sp * sr;
    x = cy * cp * sr - sy * sp * cr;
    y = sy * cp * sr + cy * sp * cr;
    z = sy * cp * cr - cy * sp * sr;
    marker.pose.orientation.x = x;
    marker.pose.orientation.y = y;
    marker.pose.orientation.z = z;
    marker.pose.orientation.w = w;

    pub->publish(marker);
}