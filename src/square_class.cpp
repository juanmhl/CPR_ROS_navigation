#include "navigation/square_class.hpp"

Square_class::Square_class()
{
    // Things to setup
    pub = nh.advertise<std_msgs::String>("cmd_square",1000);
    nh.setParam("square_length",4);
}

Square_class::~Square_class()
{
    // Things to destroy
    printf("leaving gently\n");
}

void Square_class::avanzar()
{
    std_msgs::String msg;
    msg.data = "avanzar";
    pub.publish(msg);
    ROS_INFO("Square sending: %s\n\r",msg.data.c_str());
}

void Square_class::girar()
{
    std_msgs::String msg;
    msg.data = "girar";
    pub.publish(msg);
    ROS_INFO("Square sending: %s\n\r",msg.data.c_str());
}

void Square_class::pubSquare()
{
    ros::Rate loop_rate(1); // Sending commands at 1hz
    
    int i = 1;
    
    while(ros::ok())
    {
        // Obtain square length parameter
        nh.getParam("square_length",square_length);
    
        if ((i % (square_length+1)) == 0)
        {
            girar();
        }
        else
        {
            avanzar();
        }
        
        i++;
        
        loop_rate.sleep();
    }
}
