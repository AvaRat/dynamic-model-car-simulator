#include"../includes/car_model.h"
#include<math.h>

Model::Model()
{
	//zmienne stanu dla t = 0
	yaw_rate = 0;
	lat_vel = 0;
	long_vel = 0;
	long_pos = 0;
	lat_pos = 0;
	//parametry wpisane na razie na pa³e
	r = 1;
	h = 1;
	b = 1;
	a = 1;
	mass = 10;
	i = 1;
	h = 0,05;
	g = 9,8123;
	u = 1;
	cf = cr = 1;

}

void Model::command(double torque , double steering_angle)
{
	//przeliczenie zadanego momentu na si³ê wzd³u¿n¹ ko³a
	long_for_f = torque / r;
	long_for_r = long_for_r; //zakladam rowne rozlozenie momentu na os przod/tyl

	slip_angle_f = steering_angle - (a * yaw_rate + long_vel) / lat_vel;
	slip_angle_r = (b * yaw_rate - long_vel) / lat_vel;
	norm_load_f = (mass * g * b - (long_for_f + long_for_r) * h) / (a + b);
	norm_load_r = (mass * g * a - (long_for_f + long_for_r) * h) / (a + b);
	slip_angle_est_f = cf * slip_angle_f / (u * norm_load_f);
	slip_angle_est_r = cr * slip_angle_r / (u * norm_load_r);
	lat_for_f = u * norm_load_f * (slip_angle_est_f - slip_angle_est_f * abs(slip_angle_est_f) / 3 + pow(slip_angle_est_f, 3) / 27) * sqrt(1 - pow(long_for_f / (u * norm_load_f), 2) + pow(long_for_f / cf, 2)); 
	lat_for_r = u * norm_load_r * (slip_angle_est_r - slip_angle_est_r * abs(slip_angle_est_r) / 3 + pow(slip_angle_est_r, 3) / 27) * sqrt(1 - pow(long_for_r / (u * norm_load_r), 2) + pow(long_for_r / cr, 2));

	//obliczenie zmiennych stanu x1, x2...
	yaw_rate = (a * long_for_f * steering_angle + b * lat_for_f - b * lat_for_r) / i; //x1
	lat_vel = (long_for_f * steering_angle + lat_for_f + lat_for_r) / mass - long_vel * yaw_rate; //x2
	long_vel = (long_for_f + long_for_r - lat_for_f * steering_angle) / mass - lat_vel * yaw_rate; //x3
	long_pos = -lat_vel*sin(yaw_angle) + long_vel * cos(yaw_angle); //x4
	lat_pos = lat_vel * cos(yaw_angle) + long_pos * cos(yaw_angle); //x5

	//aktualizacja zmiennych
	yaw_rate = yaw_rate * tau;
	lat_vel = lat_vel * tau;
	long_vel = long_vel * tau;
	long_pos = long_pos * tau;
	lat_pos = lat_pos * tau;
	yaw_angle = yaw_angle * tau; //x6


    std::cout << "command called\n";
}

std::vector<double> Model::get_position()
{
    std::vector<double> vec = {long_pos, lat_pos, yaw_angle};
    return vec;
}