#ifndef MAX_IO_P_H
#define MAX_IO_P_H

#include <memory>

#include <boost/asio.hpp>
#include "max_io.h"
#include "log.h"

namespace ba = boost::asio;

namespace max_io {

enum struct comstate {
    setupPort,
    receiving
};


struct ncube::Private
{
    std::shared_ptr<ba::io_service> ios;
    ba::serial_port sio;
    ba::io_service::work work;
    std::string port;
    ba::streambuf rxbuf;
    // ba::deadline_timer dlt;
    ba::steady_timer versionto;

    ba::steady_timer sampleto;

    std::string txbuf;

    comstate cstate;

    // config rw
    config cdata;
    // workdata
    std::thread thrd;
    // std::map<rfaddr, thermostat_actual_values> thermostats;
    std::map<rfaddr, thermostat_state> thermostats;
    std::map<rfaddr, thermostat_state> wallthermostats;


    std::map<rfaddr, room_id> room_map;

    callback cb;

    Private(std::shared_ptr<ba::io_service> io, callback &ncb)
        : ios(io)
        , sio(*(ios.get()))
        , work(*(ios.get()))
        , versionto(*(ios.get()))
        , sampleto(*ios)
        , cb(ncb)
    {
        L_Dbg << "ios " << ios.get();
    }
};
}

#endif // MAX_IO_P_H
