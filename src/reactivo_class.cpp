#include "navigation/reactivo_class.hpp"

Reactivo_class::Reactivo_class()
{
    sub = nh.subscribe("base_scan", 1, &Reactivo_class::base_scanCallback, this); // suscrito a topic del laser
    
    pub = nh.advertise<std_msgs::String>("/cmd_reactivo",1000); // publica a manager los comandos
}

void Reactivo_class::base_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Search for the nearest object - smaller distance in msg.ranges
    int n_ranges = msg->ranges.size();
    std::vector<float>::const_iterator min_it = std::min_element(msg->ranges.begin(),msg->ranges.end());
    double nearest = *min_it;
    int pos=std::distance(msg->ranges.begin(),min_it);
    
    ROS_INFO_STREAM("Total meassurements: " << n_ranges);
    ROS_INFO_STREAM("Nearest obstacle at: " << nearest << " at vector position: " << pos);
    
    // Nearest object not under:
    float threshold = 0.5;
    ros::Rate loop_rate(5);
    int numGiros = 16;
    
    if (nearest<threshold)
    {
        numGiros = 16 + round(31*((double)rand()/(double)RAND_MAX)); // num of gira msgs to send, random, from 16 to 47, for freq = 10
                                                                     // for freq = 5, use half the steps
        ROS_INFO_STREAM("Numero de giros: " << numGiros);
        for(int i=0; i<numGiros/2; i++) {sendGira(); loop_rate.sleep();} // send X gira msgs 0.2s appart (from 1.6 up to 4.7 s of giros) -> at 1rad/s: from pi/2 to 3pi/2
        sendAvanza();    // to leave the wall so it doesn't trigger gira again just after girar
        loop_rate.sleep(); 
    }
    else
    {
        sendAvanza();
    }
    
}

void Reactivo_class::sendAvanza()
{
    std_msgs::String msg;
    msg.data = "avanza";
    pub.publish(msg);
    ROS_INFO_STREAM("AVANZA");
}

void Reactivo_class::sendGira()
{
    std_msgs::String msg;
    msg.data = "gira";
    pub.publish(msg);
    ROS_INFO_STREAM("GIRA");
}

Reactivo_class::~Reactivo_class()
{
    ROS_INFO_STREAM("leaving gently");
}
