#include "navigation/reactivo_class.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"reactivo_node");
    
    Reactivo_class node;
    
    ros::Rate loop_rate(5); // 0.1s refresh rate
    
    // Scan reading at 10hz:    
    while(ros::ok())
    {
        ros::spinOnce(); // attend callbacks
        loop_rate.sleep();  // wait
    }
    
    return 1;
}
