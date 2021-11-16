#include "navigation/roomba_class.hpp"

Roomba_class::Roomba_class()
{
    sub = nh.subscribe("base_scan", 1, &Roomba_class::base_scanCallback, this);     // suscrito a topic del laser
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);                       // publica a stage_ros
    serverStart = nh.advertiseService("start",&Roomba_class::start_function,this);
    stopped=true;
}

void Roomba_class::base_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Search for the nearest object - smaller distance in msg.ranges
    n_ranges = msg->ranges.size();
    std::vector<float>::const_iterator min_it = std::min_element(msg->ranges.begin(),msg->ranges.end());
    nearest = *min_it;
    pos=std::distance(msg->ranges.begin(),min_it);
    
    ROS_INFO_STREAM("Total meassurements: " << n_ranges);
    ROS_INFO_STREAM("Nearest obstacle at: " << nearest << " at vector position: " << pos);
}

bool Roomba_class::start_function(navigation::start::Request& req, navigation::start::Response& res)
{
    stopped = false;
    ROS_INFO_STREAM("Start fnc activated");
    return true;
}

void Roomba_class::cmd_velPublish(const double& linear, const double& angular)
{
    geometry_msgs::Twist vel_msg;
    
    vel_msg.linear.x = linear;
    vel_msg.angular.z = angular;
    
    ROS_INFO_STREAM("Value: " << vel_msg.linear.x << " " << vel_msg.angular.z);
    pub.publish(vel_msg);
}

void Roomba_class::spiral()
{
    //ROS_INFO_STREAM("ENTER SPIRAL METHOD");
    linear = 0.5;
    angular = 2;
    ros::Rate loop_rate(f);
    
    while(stopped) {ros::spinOnce(); loop_rate.sleep();}
    
    while( (ros::ok()) and (nearest>crashThreshold) and (!stopped) )
    {
        //ROS_INFO_STREAM("INSIDE SPIRAL WHILE");
        angular = angular * 0.985;
        cmd_velPublish(linear, angular);
        loop_rate.sleep();
        ros::spinOnce();
        //ROS_INFO_STREAM("ROS::OK = " << ros::ok());
    }
}

Roomba_class::~Roomba_class()
{
    ROS_INFO_STREAM("Leaving gently...");
}
