#include "navigation/keyboard_class.hpp"

Keyboard_class::Keyboard_class()
{
    // Things to setup
    pub = nh.advertise<std_msgs::String>("cmd_key",1000);
}

Keyboard_class::~Keyboard_class()
{
    // Things to destroy
    printf("leaving gently\n");
}

void Keyboard_class::publish_key()
{
    
    //Enter in console raw mode. It is used for avoiding pressing enter after each character
    system("stty raw");
    bool salir=false;   //exit after pressing a particular key
    
    std_msgs::String msg;
    
    ROS_INFO("\n\n\r-- NAVIGATE USING THE KEYBOARD -- \n\n\r q --> forwards \n\r a --> backwards \n\r o --> left \n\r p --> right \n\r space --> stop \n\r x --> leave \n\n\r");
    
    while (ros::ok() && !salir)
    {
        char input=getchar();
        switch (input)
        {
            case 'q': msg.data = "forwards"; break;
            case 'a': msg.data = "backwards"; break;
            case 'o': msg.data = "left"; break;
            case 'p': msg.data = "right"; break;
            case 32: msg.data = "stop"; break;
            //code here
            case 'x': {salir=true;system("stty cooked");} //restore the console parameters
            default: msg.data = ""; break;
        }
        //code here
        pub.publish(msg);
        ROS_INFO("Sending key [%c] ---> %s\r",input,msg.data.c_str()); //Important to include \r to correctly print data in raw mode
    }
    
    
}
