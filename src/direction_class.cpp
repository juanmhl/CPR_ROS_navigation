#include "navigation/direction_class.hpp"

Direction_class::Direction_class()
{
    // Things to setup
    pub = nh.advertise<std_msgs::String>("cmd_dir",1000);
}

Direction_class::~Direction_class()
{
    // Things to destroy
    printf("leaving gently\n");
}

void Direction_class::publish_dir()
{
    unsigned dir=0;
    std_msgs::String msg;
    std::string ss;
    dir = round(4*((double)rand()/(double)RAND_MAX));
        switch(dir)
        {
            case 0: ss="stop";break;
            case 1: ss="forwards";break;
            case 2: ss="backwards";break;
            case 3: ss="left";break;
            case 4: ss="right";break;
        }
        
        msg.data = ss;
        ROS_INFO("%d ---> %s\n",dir,msg.data.c_str());
        pub.publish(msg);
}
