#ifndef MAX_IO_H
#define MAX_IO_H

#include <string>
#include <memory>
#include <functional>
#include <set>
#include <variant>

#include <boost/asio.hpp>

namespace max_io
{

using eval_data_fn = std::function<void (std::string &&)>;

using rfaddr = uint32_t;
using room_id = uint8_t;
using rfaddr_name = std::map<uint32_t, std::string>;

struct room
{
    room_id             id;
    std::string         name;
    rfaddr_name         thermostats;
    rfaddr              wallthermostat;
    room() {}
    room(room_id rid, const std::string &rname)
        : id(rid)
        , name(rname)
    {}
};


using config = std::map<room_id, room>;

enum struct event_e {
    Start,
    GotVersion,
    GotPacket,
    RxError,
    VersionTimeout,
};

using eventdata_t = std::variant<std::monostate,std::string>;

using event_t = std::pair<event_e, eventdata_t>;

class ncube
{
public:
    ncube(std::string port,
          const config &config);

    ncube(std::string port,
          const config &config,
          std::shared_ptr<boost::asio::io_context> ios);
private:
    void start();
    void setup_port(const boost::system::error_code& error_code, std::size_t, unsigned state);
    void sync_setup_port();
    void initialize_port();
    bool open_port();
    void enable_moritz();
    void start_packet_read();
    std::string get_received();

    void setup_workdata();
    bool is_thermostat(rfaddr rf) const;
    bool is_wallthermostat(rfaddr rf) const;

    std::string devinfo(rfaddr a);

    void sync_enable_moritz();

    void start_async_write(const std::string &txd, std::function<void (const boost::system::error_code &, std::size_t)> fn);

    void evaluate_version(std::string &&);

    void evaluate_packet(std::string &&);

    //void version_read(const boost::system::error_code& error_code,
    //                  std::size_t bytes_transferred);

    void packet_read(const boost::system::error_code& error_code,
                      std::size_t bytes_transferred);

    void process_event(const event_t &);


    void process_data(const std::string &datain);
private:
    struct Private;
    std::shared_ptr<Private> _p;
};

}

#endif // MAX_IO_H
