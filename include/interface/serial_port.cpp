#include "serial_port.h"

SerialPort::SerialPort(const string &port_name, int baudrate)
: port_name_(port_name), baudrate_(baudrate)
{

}

SerialPort::~SerialPort()
{
    serial_.close();
}

int SerialPort::read_message(mavlink_message_t &message)
{
    uint8_t cp;
    mavlink_status_t status;
    uint8_t msgReceived = false;

    // int result = 0;

    if(serial_.available())
    {
        serial_.read(&cp, 1);

        msgReceived = mavlink_parse_char(MAVLINK_COMM_0, 
                                         cp, 
                                         &message, 
                                         &status);
    }

    // if(result > 0)
    // {

        
    //     if( last_status_.packet_rx_drop_count != status.packet_rx_drop_count )
    //     {
    //         std::cout << "ERROR: DROPPED PACKETS" 
    //         << std::endl;
    //     }
    //     last_status_ = status;
    // }
    // else
    // {
    //     std::cout << 
    //     "ERROR: Could not read from serial port" 
    //     << std::endl;
    // }

    return msgReceived;
}

int SerialPort::write_message(const mavlink_message_t &message)
{
    char buf[MAVLINK_MAX_PACKET_LEN];

    unsigned int len = 
    mavlink_msg_to_send_buffer((uint8_t*)buf,
                                &message);

    size_t bytes_sent = serial_.write((uint8_t*)buf, len);                    
    return bytes_sent;
}

void SerialPort::start()
{
    try{
        serial_.setPort(port_name_);
        serial_.setBaudrate(baudrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(timeout);
    }
    catch(serial::IOException& e)
    {
        std::cout << "ERROR: Could not open serial port" 
        << std::endl;
    }

    serial_.open();

}
