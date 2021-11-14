#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

class Reactivo_class
{
public:
    
    Reactivo_class();
    
    void base_scanCallback(const sensor_msgs::LaserScan::ConstPtr & msg);
    
    ~Reactivo_class();
    
private:
    
    ros::NodeHandle nh;
    
    ros::Publisher pub;
    
    ros::Subscriber sub;
    
        
    void sendAvanza();
    
    void sendGira();
    
    
};
