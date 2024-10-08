#include "interface/generic_port.h"
#include "interface/mavlink_messages.h"
#include "interface/serial_port.h"

#include "ros_mavlink/ros_wrapper_mavlink.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_mavlink_node");

    ros::NodeHandle nh;

    SerialPort port("/dev/ttyACM0", 57600);

    RosWrapperMavlink wrapper(nh, &port);

    wrapper.ros_run();

    return 0;
}