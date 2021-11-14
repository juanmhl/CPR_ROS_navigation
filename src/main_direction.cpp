#include "navigation/direction_class.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "direction_node");
    
    Direction_class node;
    
    ros::Rate loop_rate(1);
    
    while(ros::ok())
    {
        node.publish_dir();
        loop_rate.sleep();
    }
    
    return 1;
}
