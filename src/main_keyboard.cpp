#include "navigation/keyboard_class.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"keyboard_node");
    
    Keyboard_class keyboard;
    
    keyboard.publish_key();
    
    return 1;
}
