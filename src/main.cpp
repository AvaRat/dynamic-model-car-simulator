#include<utility>
#include "../includes/dynamic_tree.h"

using namespace std;
int main(void)
{
    Path path;
    path.read_from_file("/home/marcel/Documents/Article_RTOS_2020/dynamic-model-car-simulator/symulator.txt");
    Model model;
    Dynamic_tree tree(model, path);
   // tree.create_children(make_pair(5.0,-5.0), make_pair(0.78539, -0.78539));
    tree.create_children({10.0,-5.0}, {0.78539, -0.78539});
    return 0;
}