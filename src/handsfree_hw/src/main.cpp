#include <handsfree_hw/hf_hw_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw");
    ros::NodeHandle nh("handsfree");
    std::string config_filename = "/config.txt" ;
    std::string config_filepath = CONFIG_PATH+config_filename ;

    ros::NodeHandle nh_private("~");
    std::string serial_port;
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    std::string serial_port_path="serial://" + serial_port;
    handsfree_hw::HF_HW_ros hf(nh, serial_port_path, config_filepath);
//    handsfree_hw::HF_HW_ros hf(nh, "serial:///dev/ttyUSB0", config_filepath);
    hf.mainloop();
    return 0;
}
