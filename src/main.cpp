#include "../includes/car_model.h"
#include "path.cpp"

using namespace std;
int main(void)
{
    Path path;
    path.read_from_file("/home/marcel/Documents/Article_RTOS_2020/dynamic-model-car-simulator/symulator.txt");
    return 0;
}