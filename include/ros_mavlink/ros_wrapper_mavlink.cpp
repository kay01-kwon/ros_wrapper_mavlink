#include "ros_wrapper_mavlink.hpp"

RosWrapperMavlink::RosWrapperMavlink(const ros::NodeHandle &nh, 
GenericPort *port)
:nh_(nh), port_(port), reading_status_(false)
{
}