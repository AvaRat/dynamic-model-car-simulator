#include"simulator.cpp"


using namespace std;
int main(int argc, char **argv)
{

  double dT = 0;
  double initial_speed = 0;
  ros::init(argc, argv, "simulator");
  ros::NodeHandle n;
  ros::Rate r(100);
  n.param("dT", dT, 0.01);
  n.param("initial_speed", initial_speed, 0.0);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1);
  ros::Publisher speed_pub = n.advertise<std_msgs::Float64>("speed", 1);
  Simulator simulator(pose_pub, speed_pub, dT, initial_speed);

  ros::Subscriber distance_sub = n.subscribe("s", 1, &Simulator::track_progress_callback, &simulator);
  ros::Subscriber error_pub = n.subscribe("d", 1, &Simulator::error_callback, &simulator);
  ros::Subscriber cmd_sub = n.subscribe("model_control", 10, &Simulator::cmd_callback, &simulator);
  simulator.set_cmd_subscriber(cmd_sub);

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
