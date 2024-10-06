#ifndef ROS_WRAPPER_MAVLINK_H
#define ROS_WRAPPER_MAVLINK_H

#include <time.h>
#include <sys/time.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include "interface/generic_port.h"
#include "interface/mavlink_messages.h"

class RosWrapperMavlink
{
    public:

    RosWrapperMavlink() = delete;

    RosWrapperMavlink(const RosWrapperMavlink &other) = delete;

    RosWrapperMavlink &operator=(const RosWrapperMavlink &other) = delete;

    RosWrapperMavlink(const ros::NodeHandle &nh,GenericPort *port);

    RosWrapperMavlink(const ros::NodeHandle &nh, GenericPort *port, int ros_rate);

    ~RosWrapperMavlink();

    private:

    MavlinkMessages current_messages_;

    bool reading_status_;

    int sysid_, compid_;

    GenericPort *port_;
    ros::NodeHandle nh_;
    ros::Publisher mavlink_pub_;
    ros::Rate loop_rate_{200};

    boost::thread read_thread_;
    boost::thread rosrun_thread_;

    boost::mutex mtx_;

    void start();

    void stop();

    void read_messages();

    void publish_messages();
    
};


#endif // ROS_WRAPPER_MAVLINK_H