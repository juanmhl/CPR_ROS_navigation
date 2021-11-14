#include "navigation/square_class.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_node");
    
    Square_class node;
    
    node.pubSquare();
    
    return 1;
}
