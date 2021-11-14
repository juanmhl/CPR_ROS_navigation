#include "ros/ros.h"
#include "std_msgs/String.h"

class Direction_class
{
public:
    Direction_class();
    
    void publish_dir();
    
    ~Direction_class();
    
private:
    
    ros::NodeHandle nh;
    
    ros::Publisher pub;
    
};
