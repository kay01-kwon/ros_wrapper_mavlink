#include "ros_wrapper_mavlink.hpp"

RosWrapperMavlink::RosWrapperMavlink(const ros::NodeHandle &nh,
                                     SerialPort *port)
: nh_(nh), port_(port)
{
    current_messages_.sysid = sysid_;
    current_messages_.compid = compid_;

    publisher_setup();

    port_->start();

    usleep(1000);

    mavlink_message_t message;

    current_messages_.sysid = message.sysid;
    current_messages_.compid = message.compid;

    printf("System id: %d\n", current_messages_.sysid);
    printf("Component id: %d\n", current_messages_.compid);

    mavlink_message_t message_to_write;

    mavlink_command_long_t com = {0};

    com.target_system = message.sysid;
    com.target_component = message.compid;
    com.confirmation = true;
    com.command = MAV_CMD_REQUEST_MESSAGE;
    com.param1 = MAVLINK_MSG_ID_HEARTBEAT;
    com.param2 = 1000000;

    mavlink_msg_command_long_encode(sysid_, compid_, 
    &message_to_write, &com);
    port->write_message(&message_to_write);

    com.param1 = MAVLINK_MSG_ID_HIGHRES_IMU;

    mavlink_msg_command_long_encode(sysid_, compid_, 
    &message_to_write, &com);
    port->write_message(&message_to_write);

    com.param1 = MAVLINK_MSG_ID_ATTITUDE_QUATERNION;

    mavlink_msg_command_long_encode(sysid_, compid_,
    &message_to_write, &com);
    port->write_message(&message_to_write);

    com.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    com.param1 = MAVLINK_MSG_ID_HIGHRES_IMU;
    com.param2 = 2500.0;

    mavlink_msg_command_long_encode(sysid_, compid_, 
    &message_to_write, &com);

    port->write_message(&message_to_write);

    com.param1 = MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
    mavlink_msg_command_long_encode(sysid_, compid_, 
    &message_to_write, &com);

    port->write_message(&message_to_write);


}

RosWrapperMavlink::RosWrapperMavlink(const ros::NodeHandle &nh, SerialPort *port, int ros_rate)
: nh_(nh), port_(port), reading_status_(false)
{
    current_messages_.sysid = sysid_;
    current_messages_.compid = compid_;

    loop_rate_ = ros::Rate(ros_rate);

    arm_disarm(true);

    publisher_setup();

}

void RosWrapperMavlink::ros_run()
{
    mavlink_message_t message;
    float qx, qy, qz, qw;

    std::cout << "ROS Wrapper Mavlink running..." << std::endl;

    while(ros::ok())
    {
        // boost::lock_guard<boost::mutex> lock(mtx_);
        int result = port_->read_message(&message);
        if(result)
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
                    printf("Heartbeat received\n");
                }
                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    mavlink_msg_sys_status_decode(&message, 
                    &current_messages_.sys_status);
                    break;
                    printf("System status received\n");
                }
                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    mavlink_msg_highres_imu_decode(&message, 
                    &current_messages_.highres_imu);
                    printf("Highres IMU received\n");

                    // Write IMU message data
                    imu_msg_.header.frame_id = "imu_link";
                    
                    imu_msg_.linear_acceleration.x = 
                    current_messages_.highres_imu.xacc;

                    imu_msg_.linear_acceleration.y = 
                    current_messages_.highres_imu.yacc;

                    imu_msg_.linear_acceleration.z = 
                    current_messages_.highres_imu.zacc;

                    // imu_msg_.angular_velocity.x = 
                    // current_messages_.highres_imu.xgyro;

                    // imu_msg_.angular_velocity.y = 
                    // current_messages_.highres_imu.ygyro;

                    // imu_msg_.angular_velocity.z = 
                    // current_messages_.highres_imu.zgyro;

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

                    printf("Attitude quaternion received\n");

                    qw = current_messages_.attitude_quaternion.q1;
                    qx = current_messages_.attitude_quaternion.q2;
                    qy = current_messages_.attitude_quaternion.q3;
                    qz = current_messages_.attitude_quaternion.q4;

                    imu_msg_.orientation.w = qw;

                    imu_msg_.orientation.x = qx;

                    imu_msg_.orientation.y =-qy;

                    imu_msg_.orientation.z =-qz;

                    imu_msg_.angular_velocity.x = current_messages_.attitude_quaternion.rollspeed;

                    imu_msg_.angular_velocity.y = current_messages_.attitude_quaternion.pitchspeed;

                    imu_msg_.angular_velocity.z = current_messages_.attitude_quaternion.yawspeed;

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
    arm_disarm(false);
}

void RosWrapperMavlink::publisher_setup()
{
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu",1);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/mag",1);
}

int RosWrapperMavlink::arm_disarm(bool flag)
{
    mavlink_command_long_t com = {0};

    com.target_system = sysid_;
    com.target_component = compid_;
    com.command = MAV_CMD_COMPONENT_ARM_DISARM;
    com.confirmation = 0;
    com.param1 = flag ? 1.0 : 0.1;
    com.param2 = 0.0;
    com.param3 = 0.0;
    com.param4 = 0.0;
    com.param5 = 0.0;
    com.param6 = 0.0;
    com.param7 = 0.0;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(sysid_, compid_, &message, &com);

    int len = port_->write_message(&message);

    return len;
}

int RosWrapperMavlink::toggle_offboard_control(bool flag)
{
    mavlink_command_long_t com = {0};
    com.target_system = sysid_;
    com.target_component = compid_;
    com.command = MAV_CMD_NAV_GUIDED_ENABLE;
    com.confirmation = true;
    com.param1 = flag ? 1.0 : 0.1;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(sysid_, compid_, &message, &com);

    int len = port_->write_message(&message);

    return len;
}