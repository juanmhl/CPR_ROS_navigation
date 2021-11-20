#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include "navigation/start.h"
#include "navigation/getCrashes.h"

class Roomba_class
{
public:
    
    Roomba_class();
    ~Roomba_class();
    
    // Topics
    void base_scanCallback(const sensor_msgs::LaserScan::ConstPtr & msg);
    void base_pose_ground_truthCallback(const nav_msgs::Odometry::ConstPtr & msg);
    void cmd_velPublish(const double & linear, const double & angular);
    
    // Services
    bool start_function(navigation::start::Request &req,
                        navigation::start::Response &res);
    
    bool getCrashes_function(navigation::getCrashes::Request &req,
                             navigation::getCrashes::Response &res);
    
    // Basic movement
    void spiral();
    void evade();
    void straight();
    
    void followWall();
    
    unsigned int crashes = 0; // Number of crashes evaded when using straight mode
    
private:
    
    // Basics
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber subPose;
    
    // Service servers
    ros::ServiceServer serverStart;
    ros::ServiceServer serverGetCrashes;
    
    bool stopped; // controlled by start service
    
    // Info provided by the laserScan
    int n_ranges = -1;      // laserScan vector length
    double nearest = -1;    // nearest distance detected by the laserScan
    int pos = -1;           // pos of the nearest
    
    // Pose of the robot
    double poseX = 0;
    double poseY = 0;
    double poseX_orig = 0;
    double poseY_orig = 0;
    
    double crashThreshold = 0.45;
    
    double linear = 0, angular = 0;
    
    double f = 10; // message publishing frequency
    
    // Used in followWall() method
    void getCloser();
    void getAway();
    void wallToTheRight();
    
    // Method to update crashes counters
    void updateCrash();
    
    // Number of crashes counters, accesed by getCrashes service
    int crashRight = 0;
    int crashLeft = 0;
    int crashCenter = 0;
    
};
