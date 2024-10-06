#ifndef ROS_WRAPPER_MAVLINK_H
#define ROS_WRAPPER_MAVLINK_H

#include <time.h>
#include <sys/time.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "interface/generic_port.h"
#include "interface/mavlink_messages.h"


using sensor_msgs::Imu;
using sensor_msgs::MagneticField;

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

    bool reading_status_{false};

    int sysid_{0}, compid_{0};

    GenericPort *port_;

    ros::NodeHandle nh_;

    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Rate loop_rate_{200};
    
    Imu imu_msg_;
    MagneticField mag_msg_;

    bool highres_imu_received_{false};
    bool attitude_quaternion_received_{false};

    boost::thread read_thread_;
    boost::thread rosrun_thread_;

    boost::mutex mtx_;
    boost::condition_variable cv_;

    void publisher_and_thread_setup();

    void start();

    void stop();

    void read_thread_func();

    void rosrun_thread_func();
    
};


#endif // ROS_WRAPPER_MAVLINK_H