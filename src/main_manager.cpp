#include "navigation/manager_class.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"manager_node");
    
    Manager_class manager;
    
    ros::Rate loop_rate(5); // Defino frecuencia de delay
    
    while(ros::ok())
    {
        ros::spinOnce(); // Atiendo a los topic --> callbacks
        
        if (manager.getMode()=="teclado") {manager.publishKeyboard();}

        loop_rate.sleep(); // Delay
    }
    
    
    // ros::spin();  // Esto entra en un bucle de atención a los topic --> callbacks
    
    
    return 1;
}
