#include<iostream>
#include<cassert>
#include"../includes/dynamic_tree.h"


Dynamic_tree::Dynamic_tree(Model model): dynamic_model(model), grand_parent(0, 0)
{
}

void Dynamic_tree::DFS(Node *node)
{
    
}

void Dynamic_tree::create_children(pair<double,double> momentum_max_min, pair<double, double> angle_max_min)
{
    grand_parent.create_children(momentum_max_min, angle_max_min);
}


Node::Node(double momentum, double steering_angle)
{
    children.reserve(n_children);
    command.momentum = momentum;
    command.steering_angle = steering_angle;
}

/**
 * momentum_range=x;
 * min momentum = -x
 * max momentum = x
 * 
 */
void Node::create_children(pair<double,double> momentum_max_min, pair<double, double> angle_max_min)
{
    
    assert(momentum_max_min.first > momentum_max_min.second);
    assert(angle_max_min.first > angle_max_min.second);

    double momentum_increment = (momentum_max_min.first - momentum_max_min.second) / momentum_n;
    double angle_increment = (angle_max_min.first - angle_max_min.second) / angle_n;

    vector<pair<double, double>>cmd_values = {};
    for(size_t m=0; m<=momentum_n; m++)
        for(size_t a=0; a<=angle_n; a++)
            cmd_values.push_back(make_pair(momentum_max_min.first - m*momentum_increment, \
                                            angle_max_min.first - a*angle_increment));
  //  cout << "size of cmd_values = " << cmd_values.size() << endl;
    for(size_t i=0; i<n_children; i++)
    {
     //   cout << cmd_values[i].first << "\t" << cmd_values[i].second << endl;
        Node *new_node = new Node(cmd_values[i].first, cmd_values[i].second);
        children.push_back(new_node);
    }
}

Node::~Node()
{
    for(auto it=children.begin(); it !=children.end(); it++)
        free(*it);
}