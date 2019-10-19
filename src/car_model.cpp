#include"../include/car_model.h"

// test gita!xxxxxxxxxxxxxxxxxxxxxx

Model::Model(double Dt)
{
	//zmienne stanu dla t = 0
	yaw_rate = 0;
	lat_vel = 0;
	long_vel = 0;
	long_pos = 0;
	lat_pos = 0;
	//parametry wpisane na razie na pale
  tau = Dt;
	r = 0.3;
	b = 2;
	a = 2;
	mass = 1292;
	i = 2380;
	h = 0.3;
	g = 9.8123;
	u = 0.7;
	cf = cr = 5000;

	max_torque = r * (u * ((mass*g*b/(a+b)) / (1 + (u * h / (a+b)))));


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

void Model::command(double torque , double steering_angle)
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
		//przeliczenie zadanego momentu na sile wzdluzna kola
		long_for_f = torque / r;
		long_for_r = long_for_f; //zakladam rowne rozlozenie momentu na os przod/tyl

        if(abs(long_vel) < 1e-16)
		{
        slip_angle_f = 0;
		    slip_angle_r = 0;
        }else
        {
        slip_angle_f = steering_angle - ((a * yaw_rate + lat_vel) / long_vel);
		    slip_angle_r = (b * yaw_rate - lat_vel) / long_vel;
        }



		norm_load_f = (mass * g * b - ((long_for_f + long_for_r) * h)) / (a + b);
		norm_load_r = (mass * g * a + ((long_for_f + long_for_r) * h)) / (a + b);
		slip_angle_est_f = cf * slip_angle_f / (u * norm_load_f);
		slip_angle_est_r = cr * slip_angle_r / (u * norm_load_r);
		lat_for_f = u * norm_load_f * (slip_angle_est_f - (slip_angle_est_f * abs(slip_angle_est_f) / 3) + (pow(slip_angle_est_f, 3) / 27)) * sqrt(1 - pow(long_for_f / (u * norm_load_f), 2) + pow(long_for_f / cf, 2));
		lat_for_r = u * norm_load_r * (slip_angle_est_r - (slip_angle_est_r * abs(slip_angle_est_r) / 3) + (pow(slip_angle_est_r, 3) / 27)) * sqrt(1 - pow(long_for_r / (u * norm_load_r), 2) + pow(long_for_r / cr, 2));

		//obliczenie zmiennych stanu x1, x2...
		yaw_rate = yaw_rate + ((((a * long_for_f * steering_angle) + (b * lat_for_f) - (b * lat_for_r)) / i) * tau); //x1

		lat_vel = lat_vel + (((((long_for_f * steering_angle) + lat_for_f + lat_for_r) / mass) - (long_vel * yaw_rate)) * tau); //x2

		long_vel = long_vel + ((((long_for_f + long_for_r - (lat_for_f * steering_angle)) / mass) - (lat_vel * yaw_rate)) * tau); //x3

		long_pos = long_pos + (((-lat_vel*sin(yaw_angle)) + (long_vel * cos(yaw_angle))) * tau); //x4

		lat_pos = lat_pos + (((lat_vel * cos(yaw_angle)) + (long_vel * sin(yaw_angle))) * tau); //x5

		yaw_angle = yaw_angle + (yaw_rate * tau); //x6


 //   std::cout << "car state changed\n";
}

std::vector<double> Model::get_position()
{
    std::vector<double> vec = {long_pos, lat_pos, yaw_angle};
    return vec;
}
std::map<std::string, double> Model::get_data()
{
    std::map<std::string, double> data;
    data["x"] = long_pos;
    data["y"] = lat_pos;
    data["lat_vel"] = lat_vel;
    data["long_vel"] = long_vel;
    data["yaw_angle"] = yaw_angle;
    data["yaw_rate"] = yaw_rate;
    data["slip_angle_f"] = slip_angle_f;
    data["norm_load_f"] = norm_load_f;
    data["norm_load_r"] = norm_load_r;
    data["slip_angle_est_f"] = slip_angle_est_f;
    data["slip_angle_est_r"] = slip_angle_est_r;
    return data;
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
