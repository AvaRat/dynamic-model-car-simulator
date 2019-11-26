#include"simulator.cpp"

using namespace std;
int main(int argc, char **argv)
{
    double dT = 0;
    double initial_speed = 0;
    double friction_coef = 0.9;
    ros::init(argc, argv, "simulator");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");
    ros::Rate r(100);
    pnh.param("dT", dT, 0.01);
    pnh.param("initial_speed", initial_speed, 0.0);
    pnh.param("friction_coef", friction_coef, 0.9);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("model_pose", 1);
    ros::Publisher speed_pub = n.advertise<std_msgs::Float64>("model_speed", 1);
    ros::Publisher acc_pub = n.advertise<std_msgs::Float64>("model_acceleration", 1);
    Simulator simulator(pose_pub, speed_pub, acc_pub, dT, initial_speed);  
    ros::Subscriber distance_sub = n.subscribe("s", 1, &Simulator::track_progress_callback, &simulator);
    ros::Subscriber error_pub = n.subscribe("d", 1, &Simulator::error_callback, &simulator);
    ros::Subscriber pause_sub = n.subscribe("sim_pause", 1, &Simulator::pause_callback, &simulator);
    // command subscriber
    ros::Subscriber cmd_sub = n.subscribe("model_control", 10, &Simulator::model_control_callback, &simulator);
    simulator.set_cmd_subscriber(cmd_sub);
    simulator.set_friction_coef(friction_coef);
    ROS_INFO("started simulation with \ndT = %lf\ninitial_speed = %lf", dT, initial_speed);
    char ch;
    while(ros::ok())
    {
      simulator.next_time_step();
      ros::spinOnce();
      r.sleep();
    } 
    return 0;
}
