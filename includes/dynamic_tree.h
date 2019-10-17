#include<iostream>
#include<array>
#include<iterator>

using namespace std;

class Node {
    private:
    using array_type = array<Node*, 100>;

    array_type children;
    bool visited = false;
    array_type::iterator active_child;  //points to children where we went
    array_type::iterator neighbour = nullptr;     //next node (neighbour) on the same depth (nullptr if ther's none)      
};

class Dynamic_tree {
    private:
    Node grand_parent;

    public:
    void DFS(Node *node);
};