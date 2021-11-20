#include "navigation/roomba_class.hpp"

Roomba_class::Roomba_class()
{
    sub = nh.subscribe("base_scan", 1, &Roomba_class::base_scanCallback, this);     // subscribed to laser topic
    subPose = nh.subscribe("base_pose_ground_truth", 1, &Roomba_class::base_pose_ground_truthCallback, this);  // subscribed to pose topic
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);                       // publish to stage_ros
    serverStart = nh.advertiseService("start",&Roomba_class::start_function,this);
    stopped=true;
    serverGetCrashes = nh.advertiseService("getCrashes",&Roomba_class::getCrashes_function,this);
}

Roomba_class::~Roomba_class()
{
    ROS_INFO_STREAM("Leaving gently...");
}

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------- TOPICS CALLBACK AND PUBLISHING ------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //

void Roomba_class::base_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Search for the nearest object - smaller distance in msg.ranges
    n_ranges = msg->ranges.size();
    std::vector<float>::const_iterator min_it = std::min_element(msg->ranges.begin(),msg->ranges.end());
    nearest = *min_it;
    pos=std::distance(msg->ranges.begin(),min_it);
    
    //ROS_INFO_STREAM("Total meassurements: " << n_ranges);
    //ROS_INFO_STREAM("Nearest obstacle at: " << nearest << " at vector position: " << pos);
}

void Roomba_class::base_pose_ground_truthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    poseX = msg->pose.pose.position.x;
    poseY = msg->pose.pose.position.y;
    //ROS_INFO_STREAM("POSE CALLBACK: " << poseX << " " << poseY);
}

void Roomba_class::cmd_velPublish(const double& linear, const double& angular)
{
    geometry_msgs::Twist vel_msg;
    
    vel_msg.linear.x = linear;
    vel_msg.angular.z = angular;
    
    //ROS_INFO_STREAM("Value: " << vel_msg.linear.x << " " << vel_msg.angular.z);
    pub.publish(vel_msg);
}

// ----------------------------------------------------------------------------------------------------- //
// ----------------------------------------- SERVICES METHODS ------------------------------------------ //
// ----------------------------------------------------------------------------------------------------- //

bool Roomba_class::start_function(navigation::start::Request& req, navigation::start::Response& res)
{
    stopped = false;    // Start movement
    ROS_INFO_STREAM("Start fnc activated");
    return true;
}

bool Roomba_class::getCrashes_function(navigation::getCrashes::Request& req, navigation::getCrashes::Response& res)
{
    res.right = crashRight;
    res.left = crashLeft;
    res.center = crashCenter;
    
    // ROS_INFO_STREAM("LEFT: " << res.left << "  RIGHT: " << res.right << "  CENTER: " << res.center);
    
    return true;
}

void Roomba_class::updateCrash()
{
    if (pos == 540) {crashCenter++;}
    else if (pos<540) {crashRight++;}
    else {crashLeft++;}
}

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------------- BASIC MOVEMENT METHODS --------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //

void Roomba_class::spiral()
{
    linear = 0.5;
    angular = 2;
    ros::Rate loop_rate(f);
    
    while(stopped) {ros::spinOnce(); loop_rate.sleep();}  // Read LaserScan until start service is called
    
    while( (ros::ok()) and (nearest>crashThreshold) and (!stopped) )
    {
        // Constant linear velocity and decreasing angular velocity results in spiral motion
        angular = angular * 0.99;
        cmd_velPublish(linear, angular);
        loop_rate.sleep();
        ros::spinOnce();  // attend LaserScan callback
    }
    
    updateCrash();  // robot exits while loop to evade imminent crash
    
    // Robot stops
    linear = 0; angular = 0;
    cmd_velPublish(linear,angular);
}

void Roomba_class::evade()
{
    // laser has 1081 meassurements
    //   pos <= 539 --> object to the right
    //   pos  = 540 --> object at center
    //   pos >= 541 --> object to the left
    
    ros::Rate loop_rate(f);
    
    // move back 2 steps
    linear = -0.5; angular = 0;
    cmd_velPublish(linear,angular); loop_rate.sleep();
    cmd_velPublish(linear,angular); loop_rate.sleep();
    
    // rotate
    //int numGiros = 3 + round(10*((double)rand()/(double)RAND_MAX));  // num of turn msgs to send, random, from 3 to 13, for freq = 5
    int numGiros = 6 + round(20*((double)rand()/(double)RAND_MAX));  // num of turn msgs to send, random, from 6 to 26, for freq = 10
    
    if (pos == 540)  // Obstacle just in front of the robot
    {
        // Random turn to the left or the right
        if (((double)rand()/(double)RAND_MAX)<=0.5)
        {
            // Rotate left
            for(int i=0; i<numGiros; i++)
            {
                cmd_velPublish(0,1);
                loop_rate.sleep();
            }
        }
        else
        {
            // Rotate right
            for(int i=0; i<numGiros; i++)
            {
                cmd_velPublish(0,-1);
                loop_rate.sleep();
            }
        }
    }
    else if (pos<=539)  // Obstacle to the right -> turn left
    {
        // Rotate left
        for(int i=0; i<numGiros; i++)
        {
            cmd_velPublish(0,1);
            loop_rate.sleep();
        }
    }
    else  // Obstacle to the left -> turn right
    {
        // Rotate right
        for(int i=0; i<numGiros; i++)
        {
            cmd_velPublish(0,-1);
            loop_rate.sleep();
        }
    }
    
}

void Roomba_class::straight()
{
    linear = 0.8;
    angular = 0;
    ros::Rate loop_rate(f);
    ros::spinOnce();
    
    while( (ros::ok()) and (nearest>crashThreshold) and (!stopped) )
    {
        ros::spinOnce();
        cmd_velPublish(linear, angular);
        loop_rate.sleep();
    }
    
    updateCrash();  // robot exits while loop to evade imminent crash
    crashes++;      // this variable is only updated in this method
    
    // Robot stops
    linear = 0; angular = 0;
    cmd_velPublish(linear,angular);
}

// ------------------------------- Wall following -------------------------------------- //

void Roomba_class::followWall()
{
    ros::Rate loop_rate(10);
    double in_threshold = 0.43;
    double out_threshold = 0.60;
    bool inicio = true;
    int count = 0;
    
    while( (ros::ok()) and (!stopped) )
    {
        ros::spinOnce();  // attend LaserScan (and pose) callback
        
        if(inicio)  // set initial pose
        {
            poseX_orig = poseX;
            poseY_orig = poseY;
            ROS_INFO_STREAM("ORIG POSE ASIGNATION:  " << poseX_orig << " " << poseY_orig);
            inicio = false;
        }
        else if( (sqrt(pow(poseX-poseX_orig,2)+pow(poseY-poseY_orig,2))<0.5) and (count>150) )  // time has passed and robot is near original pose
        {
            in_threshold += 0.3;
            out_threshold += 0.3;
            ROS_INFO_STREAM("NEW THRESHOLDS: " << in_threshold << "  |  " << out_threshold);
            inicio = true;
            count = 0;
        }
        
        // Correct distance to the wall
        if (nearest > out_threshold) {getCloser();}     
        else if (nearest < in_threshold) {getAway();}
        
        // Put wall to the right of the robot
        if ( (pos<170) or (pos>191) ) {wallToTheRight();}
        
        // Advance
        cmd_velPublish(0.8,0); loop_rate.sleep();
        cmd_velPublish(0.8,0); loop_rate.sleep();
        
        count++;
    }
    
}

void Roomba_class::getCloser()
{
    ros::Rate loop_rate(5);
    
    for(int i=0; i<7; i++) {cmd_velPublish(0,-1);       loop_rate.sleep();}
    for(int i=0; i<2; i++) {cmd_velPublish(0.25,0);     loop_rate.sleep();}
    for(int i=0; i<7; i++) {cmd_velPublish(0,1);        loop_rate.sleep();}
    
}

void Roomba_class::getAway()
{
    ros::Rate loop_rate(5);
    
    for(int i=0; i<7; i++) {cmd_velPublish(0,1);        loop_rate.sleep();}
    for(int i=0; i<2; i++) {cmd_velPublish(0.25,0);     loop_rate.sleep();}
    for(int i=0; i<7; i++) {cmd_velPublish(0,-1);       loop_rate.sleep();}
    
}

void Roomba_class::wallToTheRight()
{
    ros::Rate loop_rate(10); // max f of base_scan
    linear = 0;
    angular = 0;
    int count = 0;
    
    while( ((pos<170) or (pos>191)) and (count < 40) )  // This action will take up to 4 seconds
    {
        ros::spinOnce();                        // attend LaserScan callback
        if (pos>200)        {angular = 0.5;}    // wall far to the front            --> turn left, fast
        else if (pos>191)   {angular = 0.05;}   // wall not too much to the front   --> turn left, slow
        else if (pos<160)   {angular = -0.5;}   // wall far to the back             --> turn right, fast
        else if (pos<170)   {angular = -0.05;}  // wall not too much to the back    --> turn left, slow
        else                {angular = 0;}      // don't turn
        cmd_velPublish(linear,angular);
        loop_rate.sleep();
        count++;
    }
}
