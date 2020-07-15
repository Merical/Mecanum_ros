#include <sc_hw/sc_hw_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw");
//    ros::NodeHandle nh("handsfree");
    ros::NodeHandle nh("sc");
    std::string config_filename = "/config.txt" ;
    std::string config_filepath = CONFIG_PATH+config_filename ;

    ros::NodeHandle nh_private("~");
    std::string serial_port;
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    std::string serial_port_path="serial://" + serial_port;
    smartcar_hw::SC_HW_ros sc(nh, serial_port_path, config_filepath);
    sc.mainloop();
    return 0;
}
