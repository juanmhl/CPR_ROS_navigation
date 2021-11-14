#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

class Manager_class
{
public:
    Manager_class();
    
    void cmd_dirCallback(const std_msgs::String::ConstPtr & msg);
    
    void cmd_sourceCallback(const std_msgs::String::ConstPtr & msg);
    
    void cmd_squareCallback(const std_msgs::String::ConstPtr & msg);
    
    void cmd_keyCallback(const std_msgs::String::ConstPtr & msg);
    
    void cmd_reactivoCallback(const std_msgs::String::ConstPtr & msg);
    
    std::string getMode();
    
    void publishKeyboard();
    
    ~Manager_class();
    
private:
    
    ros::NodeHandle nh;
    
    ros::Publisher pub;
    
    ros::Subscriber subSource;
    ros::Subscriber subDir;
    ros::Subscriber subSquare;
    ros::Subscriber subKey;
    ros::Subscriber subReactivo;
    
    std::string mode = "reactivo";
    
    double linear = 0, angular = 0, inc = 0.25;
};
