#ifndef ROS_NODE_H
#define ROS_NODE_H

class Node{
    public :
        
        Node(float, float);
        Node(Node&);
        Node* parent_node;
        Node* curr_node;
        ~Node();
    private:
        float x;
        float y;
        float theta;
        float dist;

};

Node::Node(float x_pos, float y_pos)
{
    x = x_pos;
    y = y_pos;
    theta = 0;
    dist = 0;
    parent_node = nullptr;
    curr_node = new Node(x, y);
}

Node::Node(Node& node){
    parent_node = node.curr_node;
}

Node::~Node(){
    delete parent_node;
    delete curr_node;
}

#endif