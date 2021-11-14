#include "ros/ros.h"
#include "std_msgs/String.h"

class Square_class
{
public:
    
    Square_class();
    
    void pubSquare();
    
    ~Square_class();
    

    
private:
    
    ros::NodeHandle nh;
    
    ros::Publisher pub;
    
    void avanzar();
    
    void girar();
    
    int square_length;
};
