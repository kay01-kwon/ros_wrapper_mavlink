#include "ros_wrapper_mavlink.hpp"

RosWrapperMavlink::RosWrapperMavlink(const ros::NodeHandle &nh, 
GenericPort *port)
:nh_(nh), port_(port)
{
    current_messages_.sysid = sysid_;
    current_messages_.compid = compid_;

    arm_disarm(true);

    publisher_setup();

    start();
}

RosWrapperMavlink::RosWrapperMavlink(const ros::NodeHandle &nh, GenericPort *port, int ros_rate)
: nh_(nh), port_(port), reading_status_(false)
{
    current_messages_.sysid = sysid_;
    current_messages_.compid = compid_;

    loop_rate_ = ros::Rate(ros_rate);

    arm_disarm(true);

    publisher_setup();

    start();

}

void RosWrapperMavlink::ros_run()
{
    mavlink_message_t message;
    float qx, qy, qz, qw;

    while(ros::ok())
    {
        // boost::lock_guard<boost::mutex> lock(mtx_);
        if(port_->read_message(message))
        {
            current_messages_.sysid = message.sysid;
            current_messages_.compid = message.compid;

            switch(message.msgid)
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    mavlink_msg_heartbeat_decode(&message, 
                    &current_messages_.heartbeat);
                    break;
                }
                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    mavlink_msg_sys_status_decode(&message, 
                    &current_messages_.sys_status);
                    break;
                }
                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    mavlink_msg_highres_imu_decode(&message, 
                    &current_messages_.highres_imu);

                    // Write IMU message data
                    imu_msg_.header.frame_id = "imu_link";
                    
                    imu_msg_.linear_acceleration.x = 
                    current_messages_.highres_imu.xacc;

                    imu_msg_.linear_acceleration.y = 
                    current_messages_.highres_imu.yacc;

                    imu_msg_.linear_acceleration.z = 
                    current_messages_.highres_imu.zacc;

                    imu_msg_.angular_velocity.x = 
                    current_messages_.highres_imu.xgyro;

                    imu_msg_.angular_velocity.y = 
                    current_messages_.highres_imu.ygyro;

                    imu_msg_.angular_velocity.z = 
                    current_messages_.highres_imu.zgyro;

                    // Write Magnetic Field message data
                    mag_msg_.header.stamp = ros::Time::now();
                    mag_msg_.header.frame_id = "imu_link";

                    mag_msg_.magnetic_field.x =
                    current_messages_.highres_imu.xmag;

                    mag_msg_.magnetic_field.y =
                    current_messages_.highres_imu.ymag;

                    mag_msg_.magnetic_field.z =
                    current_messages_.highres_imu.zmag;

                    highres_imu_received_ = true;

                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                {
                    mavlink_msg_attitude_quaternion_decode(&message, 
                    &current_messages_.attitude_quaternion);

                    qw = current_messages_.attitude_quaternion.q1;
                    qx = current_messages_.attitude_quaternion.q2;
                    qy = current_messages_.attitude_quaternion.q3;
                    qz = current_messages_.attitude_quaternion.q4;

                    imu_msg_.orientation.w = 
                    qw;

                    imu_msg_.orientation.x = 
                    qx;

                    imu_msg_.orientation.y =
                    -qy;

                    imu_msg_.orientation.z =
                    -qz;

                    attitude_quaternion_received_ = true;

                    break;
                }
                default:
                    break;
            }   // End of switch
        }   // End of if

        if(highres_imu_received_ && attitude_quaternion_received_)
        {
            imu_msg_.header.stamp = ros::Time::now();
            highres_imu_received_ = false;
            attitude_quaternion_received_ = false;
            imu_pub_.publish(imu_msg_);
            mag_pub_.publish(mag_msg_);
        }
    }   // End of while

}

RosWrapperMavlink::~RosWrapperMavlink()
{
    port_->stop();
}

void RosWrapperMavlink::publisher_setup()
{
    port_->start();
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu",1);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/mag",1);
}

int RosWrapperMavlink::arm_disarm(bool flag)
{
    mavlink_command_long_t com = {0};

    com.target_system = sysid_;
    com.target_component = compid_;
    com.command = MAV_CMD_COMPONENT_ARM_DISARM;
    com.confirmation = true;
    com.param1 = flag ? 1 : 0;
    com.param2 = 21196;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(sysid_, compid_, &message, &com);

    int len = port_->write_message(message);

    return len;
}

void RosWrapperMavlink::start()
{
    if(!port_->is_running())
    {
        printf("Error: Port is not opened\n");
        exit(1);
    }
}