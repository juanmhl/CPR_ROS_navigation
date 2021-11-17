#include "navigation/roomba_class.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"roomba_node");
    
    Roomba_class node;
    
    node.spiral();
    node.evade();
    while(node.crashes<1) {node.straight(); node.evade();}
    node.followWall();
    
    return 0;
}
