#include"../includes/car_model.h"

Model::Model()
{

}

void Model::command(double speed, double steering_angle)
{
    std::cout << "command called\n";
}

/**
 * @retval vector<double> = {x_pos, y_pos, yaw}
 */
std::vector<double> Model::get_position()
{
    std::vector<double> vec = {long_pos, lat_pos, yaw_angle};
    return vec;
}