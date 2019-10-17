#include<iostream>
#include<vector>
#include<iterator>
#include"../includes/car_model.h"



using namespace std;


struct Command{
    double momentum;
    double steering_angle;
};


class Node {
    using array_type = vector<Node*>;

    private:

    Command command;    // actual command, that node represents

    static const size_t angle_n = 10;
    static const size_t momentum_n = 10;
    static const size_t n_children = (angle_n+1) * (momentum_n+1);   // ilość różnych komend dla jednego kroku
    array_type children;
    bool visited = false;
    array_type::iterator active_child;  //points to children where we went
    array_type::iterator neighbour;     //next node (neighbour) on the same depth (nullptr if ther's none)

    public:
    ~Node();
    Node(double momentum, double steering_angle);
    void create_children(pair<double,double> momentum_max_min, pair<double, double> angle_max_min);
    bool check();  
};

class Dynamic_tree {
    private:
    Model dynamic_model;
    Node grand_parent;

    public:
    Dynamic_tree(Model model);
    void create_children(pair<double,double> momentum_max_min, pair<double, double> angle_max_min);
    void DFS(Node *node);
};