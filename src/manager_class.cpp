#include "navigation/manager_class.hpp"

Manager_class::Manager_class()
{
    
    subSource = nh.subscribe ("cmd_source", 1000, &Manager_class::cmd_sourceCallback, this);
    subDir = nh.subscribe ("cmd_dir", 1000, &Manager_class::cmd_dirCallback, this);
    subSquare = nh.subscribe ("cmd_square", 1000, &Manager_class::cmd_squareCallback, this);
    subKey = nh.subscribe ("cmd_key", 1000, &Manager_class::cmd_keyCallback, this);
    subReactivo = nh.subscribe ("cmd_reactivo", 1000, &Manager_class::cmd_reactivoCallback, this);
    
    //pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000); // publica a turtlesim
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000); // publica a stage_ros
    
}

Manager_class::~Manager_class()
{
    // Things to destroy
    printf("leaving gently\n");
}

void Manager_class::cmd_sourceCallback(const std_msgs::String::ConstPtr & msg)
{
    mode = msg->data.c_str();
}

void Manager_class::cmd_dirCallback(const std_msgs::String::ConstPtr & msg)
{
    if (mode == "aleatorio")
    {
        ROS_INFO("[Object Manager] Recieved the direction [%s]\n", msg->data.c_str());
        geometry_msgs::Twist vel_msg;
        //double linear = 0, angular = 0;
        
        if (msg->data=="stop")              {linear=0;angular=0;}
        else if (msg->data=="forwards")     {linear=1;angular=0;}
        else if (msg->data=="backwards")    {linear=-1;angular=0;}
        else if (msg->data=="left")         {linear=0;angular=1;}
        else if (msg->data=="right")        {linear=0;angular=-1;}
        
        vel_msg.linear.x = linear;
        vel_msg.angular.z = angular;
        
        ROS_INFO_STREAM("Value: " << vel_msg.linear.x << " " << vel_msg.angular.z);
        
        pub.publish(vel_msg);
    }
}

void Manager_class::cmd_squareCallback(const std_msgs::String::ConstPtr & msg)
{
    if (mode == "cuadrado")
    {
        ROS_INFO("[Object Manager] Recieved the direction [%s]\n", msg->data.c_str());
        geometry_msgs::Twist vel_msg;
        //double linear = 0, angular = 0;
        
        if (msg->data=="avanzar")       {linear=1;angular=0;}
        else if (msg->data=="girar")    {linear=0;angular=1.57079632679;} // roughly pi/2
        
        vel_msg.linear.x = linear;
        vel_msg.angular.z = angular;
        
        ROS_INFO_STREAM("Value: " << vel_msg.linear.x << " " << vel_msg.angular.z);
        
        pub.publish(vel_msg);
    }
}

void Manager_class::cmd_keyCallback(const std_msgs::String::ConstPtr & msg)
{
    
    if (mode == "teclado")
    {
        ROS_INFO("[Object Manager] Recieved the direction [%s]\n", msg->data.c_str());
        
        
        if (msg->data=="stop")              {linear=0;angular=0;}
        else if (msg->data=="forwards")     {linear+=inc;}
        else if (msg->data=="backwards")    {linear-=inc;}
        else if (msg->data=="left")         {angular+=inc;}
        else if (msg->data=="right")        {angular-=inc;}
                
    }
}

std::string Manager_class::getMode()
{
    return mode;
}

void Manager_class::publishKeyboard()
{
    geometry_msgs::Twist vel_msg;
    
    vel_msg.linear.x = linear;
    vel_msg.angular.z = angular;
    
    ROS_INFO_STREAM("Value: " << vel_msg.linear.x << " " << vel_msg.angular.z);
    pub.publish(vel_msg);
}

void Manager_class::cmd_reactivoCallback(const std_msgs::String::ConstPtr & msg)
{
    if (mode == "reactivo")
    {
        ROS_INFO("[Object Manager] Recieved the direction [%s]\n", msg->data.c_str());
        
        if (msg->data=="avanza")        {linear=0.5; angular=0;}
        else if (msg->data=="gira")     {linear=0; angular=1;}
        
        geometry_msgs::Twist vel_msg;
    
        vel_msg.linear.x = linear;
        vel_msg.angular.z = angular;
        
        ROS_INFO_STREAM("Value: " << vel_msg.linear.x << " " << vel_msg.angular.z);
        pub.publish(vel_msg);
    }
}




