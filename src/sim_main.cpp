#include"simulator.cpp"


using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator");
    ros::NodeHandle n;
    ros::Rate r(50);
    
    Simulator simulator(n, 0.01); 



  return 0;    
}
