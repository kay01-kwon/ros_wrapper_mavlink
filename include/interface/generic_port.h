#ifndef GENERIC_PORT_H
#define GENERIC_PORT_H

#include "c_library_v2/common/mavlink.h"
#include <iostream>
#include <string>

using std::cout;
using std::endl;
using std::string;


class GenericPort{

    public:
        GenericPort() = default;

        virtual ~GenericPort() = default;

        virtual int read_message(mavlink_message_t &message) = 0;

        virtual int write_message(const mavlink_message_t &message) = 0;

        virtual bool is_running() = 0;

        virtual void start() = 0;

        virtual void stop() = 0;

};



#endif // GENERIC_PORT_H