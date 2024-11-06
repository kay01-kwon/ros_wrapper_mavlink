#ifndef ROS_WRAPPER_MAVLINK_H
#define ROS_WRAPPER_MAVLINK_H

#include <time.h>
#include <sys/time.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "interface/serial_port.h"
#include "interface/mavlink_messages.h"


using sensor_msgs::Imu;
using sensor_msgs::MagneticField;

class RosWrapperMavlink
{
    public:

    RosWrapperMavlink() = delete;

    RosWrapperMavlink(const RosWrapperMavlink &other) = delete;

    RosWrapperMavlink &operator=(const RosWrapperMavlink &other) = delete;

    RosWrapperMavlink(const ros::NodeHandle &nh,SerialPort *port);

    RosWrapperMavlink(const ros::NodeHandle &nh, SerialPort *port, int ros_rate);

    void ros_run();

    ~RosWrapperMavlink();

    private:

    MavlinkMessages current_messages_;

    bool reading_status_{false};

    int sysid_{0}, compid_{0};

    SerialPort *port_;

    ros::NodeHandle nh_;

    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Rate loop_rate_{200};
    
    Imu imu_msg_;
    MagneticField mag_msg_;

    bool highres_imu_received_{false};
    bool attitude_quaternion_received_{false};

    void publisher_setup();

    int arm_disarm(bool flag);

    int toggle_offboard_control(bool flag);
    
};


#endif // ROS_WRAPPER_MAVLINK_H