#include"simulator.cpp"


void test_callback(const std_msgs::Float32 msg)
{
  cout << "command received\n";
}

using namespace std;
int main(int argc, char **argv)
{
  
  double dT = 0;
  ros::init(argc, argv, "simulator");
  ros::NodeHandle n;
  ros::Rate r(10);
  n.param("dT", dT, 0.01);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1);
  ros::Publisher speed_pub = n.advertise<std_msgs::Float64>("speed", 1);
  Simulator simulator(pose_pub, speed_pub, dT);

  ros::Subscriber distance_sub = n.subscribe("s", 1, &Simulator::distance_callback, &simulator);
  ros::Subscriber error_pub = n.subscribe("d", 1, &Simulator::error_callback, &simulator);
  ros::Subscriber cmd_sub = n.subscribe("model_control", 1000, &Simulator::cmd_callback, &simulator);
  simulator.set_cmd_subscriber(cmd_sub);

  ROS_INFO("started simulation with dT = %lf", dT);
  ros::spin();


  return 0;    
}

