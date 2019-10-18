#include<iostream>
#include<vector>
#include<iterator>
#include"car_model.h"
#include"path.h"



using namespace std;


struct Command{
    double momentum;
    double steering_angle;
    int test_int;
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
    array_type::iterator active_child = children.begin();  //points to children where we went
    array_type::iterator neighbour = children.end();     //next node (neighbour) on the same depth (nullptr if ther's none)

    public:
    ~Node();
    Node(double momentum, double steering_angle);
    void create_children(pair<double,double> momentum_max_min, pair<double, double> angle_max_min);
    Command get_cmd();
    array_type::iterator get_active_child();
    array_type::iterator get_neighbour();
    array_type get_children();
};

class Dynamic_tree {
    private:
    Model *dynamic_model;
    Node grand_parent;
    Path *path;

    public:
    Dynamic_tree(Model *model, Path *path);

    bool check_position()
    {
        auto position_vector = dynamic_model->get_position();
        assert(position_vector.size() == 3);
        double x_pos = position_vector[0];
        double y_pos = position_vector[1];
        double yaw_angle = position_vector[2];
        
        return false; //TODO
    }
    void update_model(Command cmd);
    void create_children(pair<double,double> momentum_max_min, pair<double, double> angle_max_min);
    void DFS(Node *node);
};