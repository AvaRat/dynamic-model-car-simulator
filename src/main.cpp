#include<utility>
#include<fstream>
#include<vector>
#include "../include/dynamic_tree.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>


void parse_description(fstream &file)
{
    file << "x" << "," << "y" << "," << "t" << "," <<  "torque," << "steering_angle" << "," << "long_vel" \
        << "," << "lat_vel" << "," << "yaw_angle" << "," << "yaw_rate," << "slip_angle_f," \
        << "slip_angle_r," << "norm_load_f," << "norm_load_r," << "slip_angle_est_f," << "slip_angle_est_r," <<"lat_for_f, " << "lat_for_r" << endl;
}

void parse_data(fstream &file, Model &model, double time)
{
    auto data = model.get_data();
    file << data["x"]<<","<<data["y"]<<","<<time<<"," << data["torque"] <<"," << data["steering_angle"] << ","<< data["long_vel"]<<","<< data["lat_vel"]<<",";
    file << data["yaw_angle"]<<","<< data["yaw_rate"]<<","<< data["slip_angle_f"]<<"," <<data["slip_angle_r"] << ",";
    file << data["norm_load_f"]<<","<< data["norm_load_r"]<<","<< data["slip_angle_est_f"]<<"," ;
    file << data["slip_angle_est_r"] << "," << data["lat_for_f"] << "," << data["lat_for_r"] << endl;
}

void turn(double angle, double time)
{

}

using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator");
    ros::NodeHandle n;
    ros::Rate r(50);
    ros::Publisher pose_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    double dT = 0.001;
    size_t n_cmd = 10000; 
    
    
    double steering_angle_1 = 0;
    double torque_1 = 0;
    int steering_1_duration = 0;  //s
    int torque_1_duration = 0;  

    double steering_angle_2 = 0;
    double torque_2 = 0;
    int steering_2_duration = 0;  //s
    int torque_2_duration = 0;

    double steering_angle_3 = 0;
    double torque_3 = 0;
    int steering_3_duration = 0;  //s
    int torque_3_duration = 0;

    double steering_change_duration = 0.8;  //[seconds]

    double steering_tmp = 0;

    fstream param_file;
    cout << "opening params.txt ...";
    param_file.open("/home/marcel/Documents/Article_RTOS_2020/ros_ws/src/dynamic-model-car-simulator/params.txt", fstream::in);
    if(param_file.is_open())
        cout << "OK\n";
    else 
    {
      cout << "failed\n";
      return -1;
    }

    map<string, double> params;
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
    steering_1_duration = params["steering_1_duration"];
    steering_2_duration = params["steering_2_duration"];
    steering_3_duration = params["steering_3_duration"];
    dT = params["dT"];
    torque_1 = params["torque_1"];
    torque_2 = params["torque_2"];
    torque_3 = params["torque_3"];
    torque_1_duration = params["torque_1_duration"];
    torque_2_duration = params["torque_2_duration"];
    torque_3_duration = params["torque_3_duration"];

    steering_change_duration = params["steering_change_duration"];
    size_t steering_change_n_cmd = (size_t)(steering_change_duration/dT);
    

    param_file.close();

    Model model(dT);

    cout << "opening data file...";
    fstream data_file;
    data_file.open("/home/marcel/Documents/Article_RTOS_2020/ros_ws/src/dynamic-model-car-simulator/data.csv", fstream::out);
    if(data_file.is_open())
        cout << "OK\n";
    else 
    {
      cout << "failed\n";
      return -1;
    }
    parse_description(data_file);

    int count = 0;

    n_cmd = max((steering_1_duration+steering_2_duration + steering_3_duration)/dT, (torque_1_duration+torque_2_duration+torque_3_duration)/dT);

    vector<double> angle_cmd_vec = {steering_angle_1, steering_angle_2, steering_angle_3, 0};
    auto angle_it = angle_cmd_vec.begin();

    vector<double> torque_cmd_vec = {torque_1, torque_2, torque_3, 0};
    auto torque_it = torque_cmd_vec.begin();

    vector<size_t> angle_change = {(size_t) (steering_1_duration/dT)};
    angle_change.push_back(angle_change[0] + (size_t) (steering_2_duration/dT));
    angle_change.push_back(angle_change[1] + (size_t) (steering_3_duration/dT));
    auto angle_change_it = angle_change.begin();

    vector<size_t> torque_change = {(size_t) (torque_1_duration/dT)};
    torque_change.push_back(torque_change[0] + (size_t) (torque_2_duration/dT));
    torque_change.push_back(torque_change[1] + (size_t) (torque_3_duration/dT));
    auto torque_change_it = torque_change.begin();

    model.command(0,0);
    parse_data(data_file, model, 0);
    //main loop
    for(size_t n=0; n< n_cmd; n++)
    {   
        if(n == *torque_change_it)
        {
            torque_it++;
            torque_change_it++;
            cout <<"torque change\n";
        }
        if(n == *angle_change_it)
        {
           // cout <<"ok\n";
            double steering_increment = (*angle_it - *(angle_it+1))/steering_change_n_cmd;
            for(size_t i=0; i<steering_change_n_cmd; i++)
            {
               // cout <<"ok\n";
                model.command(*torque_it, (*angle_it)-(steering_increment*i));
                parse_data(data_file, model, n*dT);
                n++;
            }
            angle_it++;
            angle_change_it++;
        }
        
        model.command(*torque_it, *angle_it);
        parse_data(data_file, model, n*dT);
        count = 0;
        model.publish_pose(&pose_pub);
       // r.sleep();
    
    }


    cout << "closing file ... ";
    data_file.close();
    if(!data_file.is_open())
        cout << "OK\n";
    return 0;
}
