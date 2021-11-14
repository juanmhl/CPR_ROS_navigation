#include "ros/ros.h"
#include "std_msgs/String.h"

class Keyboard_class
{
public:
    Keyboard_class();
    
    void publish_key();
    
    ~Keyboard_class();
    
private:
    
    ros::NodeHandle nh;
    
    ros::Publisher pub;
    
};
