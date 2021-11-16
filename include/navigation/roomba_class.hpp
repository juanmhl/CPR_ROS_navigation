#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "navigation/start.h"

class Roomba_class
{
public:
    
    Roomba_class();
    
    void base_scanCallback(const sensor_msgs::LaserScan::ConstPtr & msg);
    
    bool start_function(navigation::start::Request &req,
                        navigation::start::Response &res);
    
    void cmd_velPublish(const double & linear, const double & angular);
    
    void spiral();
    
    void evade();
    
    ~Roomba_class();
    
    
private:
    
    ros::NodeHandle nh;
    
    ros::Publisher pub;
    
    ros::Subscriber sub;
    
    ros::ServiceServer serverStart;
    
    bool stopped;
    
    int n_ranges = -1;      // laserScan vector length
    double nearest = -1;    // nearest distance detected by the laserScan
    int pos = -1;           // pos of the nearest
    
    double crashThreshold = 0.45;
    
    double linear = 0, angular = 0;
    
    double f = 5;   // message publishing frequency
    
};
