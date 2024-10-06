#include "serial_port.h"

SerialPort::SerialPort()
{
    initialize_defaults();
}

SerialPort::SerialPort(const string &port_name, int baudrate)
{
    initialize_defaults();
    port_name_ = port_name;
    baudrate_ = baudrate;
}

SerialPort::~SerialPort()
{
    if(is_open_)
    {
        close(fd_);
    }
}

int SerialPort::read_message(mavlink_message_t &message)
{
    uint8_t cp;
    mavlink_status_t status;
    uint8_t msgReceived = false;

    int result = read_port(cp);

    if(result > 0)
    {
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, 
        cp, &message, &status);

        // Check for dropped packets
        if( (last_status_.packet_rx_drop_count != status.packet_rx_drop_count))
        {
            cout << "Error: Dropped packets" << endl;
        }
        last_status_ = status;
    }
    else
    {
        cout << "Error: Could not read from port" << endl;
    }

    return msgReceived;
}

bool SerialPort::is_running()
{
    return is_open_;
}

void SerialPort::start()
{
    cout << "Open Port" << endl;

    fd_ = open_port(port_name_);

    if(fd_ == -1)
    {
        cout << "Error: Could not open port" << endl;
        return;
    }

    bool success = setup_port(baudrate_, 8, 1, 
    false, false);

    if(!success)
    {
        cout << "Error: Could not setup port" << endl;
        return;
    }

    is_open_ = true;

    return;
}

void SerialPort::stop()
{
    if(is_open_)
    {
        close(fd_);
        is_open_ = false;
    }
    else
    {
        cout << "Error: Port is not open" << endl;
    }
}

void SerialPort::initialize_defaults()
{
    debug_ = false;
    fd_ = -1;
    is_open_ = false;

    // Initialize port settings
    port_name_ = "/dev/ttyUSB0";
    baudrate_ = 57600;

}

int SerialPort::open_port(const string &port)
{
    cout << "Opening port to " << port << endl;

    // Open serial port
    // O_RDWR: Read/Write access to serial port
    // O_NOCTTY: No terminal will control the process
    // O_NDELAY: Non-blocking mode

    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if(fd_ == -1)
    {
        cout << "Error: Unable to open port" << endl;
        return -1;
    }
    else
    {
        cout << "Port opened" << endl;
        fcntl(fd_, F_SETFL, 0);
    }

    return fd_;
}

bool SerialPort::setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
    if(!isatty(fd_))
    {
        cout << "Error: "
        << "File descriptor "
        << fd_ <<"is not a serial port. \n";
        return false;
    }

    struct termios config;

    if(tcgetattr(fd_, &config) < 0)
    {
        cout << "Error: Could not get port attributes" << endl;
        return false;
    }

    // Input flags
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | 
                        ISTRIP | IXON);

    // Output flags
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | 
                        ONOCR | OFILL | OPOST);

    // Control flags
    #ifdef OLCUC
    config.c_oflag &= ~OLCUC;
    #endif

    #ifdef ONOEOT
    config.c_oflag &= ~ONOEOT;
    #endif

    config.c_lflag &= ~(ECHO | ECHONL | 
                        ICANON | IEXTEN | ISIG);

    config.c_cflag &= ~(CSIZE | PARENB);

    config.c_lflag |= CS8;

    config.c_cc[VMIN] = 1;
    config.c_cc[VTIME] = 10;

    // Set baud rate
    switch(baud)
    {
        case 4800:
            cfsetispeed(&config, B4800);
            cfsetospeed(&config, B4800);
            break;
        case 9600:
            cfsetispeed(&config, B9600);
            cfsetospeed(&config, B9600);
            break;
        case 19200:
            cfsetispeed(&config, B19200);
            cfsetospeed(&config, B19200);
            break;
        case 38400:
            cfsetispeed(&config, B38400);
            cfsetospeed(&config, B38400);
            break;
        case 57600:
            cfsetispeed(&config, B57600);
            cfsetospeed(&config, B57600);
            break;
        case 115200:
            cfsetispeed(&config, B115200);
            cfsetospeed(&config, B115200);
            break;
        default:
            cout << "Error: Invalid baud rate" << endl;
            return false;
    }

    if(tcsetattr(fd_, TCSAFLUSH, &config) < 0)
    {
        cout << "Error: Could not set port attributes" << endl;
        return false;
    }

    return true;
}

int SerialPort::read_port(uint8_t &cp)
{
    int result = static_cast<int>(read(fd_, &cp, 1));
    return result;
}