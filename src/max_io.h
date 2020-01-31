#ifndef MAX_IO_H
#define MAX_IO_H

#include <string>
#include <memory>
#include <functional>
#include <set>
#include <variant>
#include <list>
#include <optional>
#include <boost/asio.hpp>

namespace max_io
{

using eval_data_fn = std::function<void (std::string &&)>;

using rfaddr = uint32_t;
using room_id = uint16_t;
using rfaddr_name = std::map<rfaddr, std::string>;

struct room_config
{
    room_id             id;
    std::string         name;
    rfaddr_name         thermostats;
    rfaddr              wallthermostat = { 0 };
    room_config() {}
    room_config(room_id rid, const std::string &rname)
        : id(rid)
        , name(rname)
    {}
};


struct thermostat_state;

using config = std::map<room_id, room_config>;

enum struct event_e;

using eventdata_t = std::variant<std::monostate,std::string>;

using event_t = std::pair<event_e, eventdata_t>;


/////////////////////////////
/// callback data types

enum struct opmode:uint16_t {
    automatic       = 0,
    manual          = 1,
    temporary       = 2,
    boost           = 3,
};

std::ostream &operator<<(std::ostream &s, const opmode &m);


using tstamp = std::chrono::time_point<std::chrono::system_clock>;

struct thermostat_state
{
    std::string name;
    float  desired = { 0.0 };
    float  measured = { 0.0 };
    uint16_t valve = { 0 }; // 0..100
    opmode mode = { opmode::automatic };
    bool   dst_active = { true };
    bool   link_valid = { true };
    bool   panel_locked = { false };
    bool   rf_err = { false };
    bool   battery_ok = { true };

    tstamp updated;

    thermostat_state(std::string name);
    bool operator!=(const thermostat_state &ts);
};

struct room_state
{
    std::string name;
    float desired = { 0.0 };
    float measured = { 0.0 };   

    unsigned valve;     // average over thermostats

#if 0
    using v_tstate = std::pair<
                            unsigned,           // update counter
                            thermostat_state    // state
                            >;
#endif
    std::list<thermostat_state> thermostats;

    std::optional<thermostat_state> wallthermostat;
    // skip wall thermostats for now
};

struct system_state
{
    // using versioned_roomstate = std::pair<unsigned, room_state>;
    std::list<room_state> roomstates;
};

using callback = std::function<void (std::shared_ptr<system_state>)>;


class ncube
{
public:

//     using callback = std::function< void ( std::shared_ptr<system_state> ) >;

    ncube(std::string port,
          const config &config,
          callback f);

    ncube(std::string port,
          const config &config,
          std::shared_ptr<boost::asio::io_context> ios,
          callback f);
private:
    void start();
    void sync_setup_port();
    bool open_port();
    void write_sync(std::string data);
    void enable_moritz();
    void start_packet_read();
    std::string get_received();

    void setup_workdata();
    bool is_thermostat(rfaddr rf) const;
    bool is_wallthermostat(rfaddr rf) const;

    std::string devinfo(rfaddr a);

    void sync_enable_moritz();

    void start_async_write(std::string txd, std::function<void (const boost::system::error_code &, std::size_t)> fn);

    void evaluate_packet(std::string &&);

    //void version_read(const boost::system::error_code& error_code,
    //                  std::size_t bytes_transferred);

    void packet_read(const boost::system::error_code& error_code,
                      std::size_t bytes_transferred);

    void process_event(const event_t &);


    void process_data(std::string &&datain);

    bool room_from_rfaddr(rfaddr src, unsigned &roomnr);    

    void update_thermostat_mode_valve_desired(rfaddr src, opmode m, uint16_t valve, float desired);
    void update_thermostat_mode_valve_desired_measured(rfaddr src, opmode m, uint16_t valve, float desired, float measured);
    void update_wallthermostat_desired_measured(rfaddr src, float desired, float measured);

    void emit_update();

    bool set_mode_valve_desired(thermostat_state &, opmode m, uint16_t valve, float desired);
    bool set_measured_desired(thermostat_state &, float desired, float measured);
    bool set_mode_desired(thermostat_state &, opmode mode, float measured);    

private:
    struct Private;
    std::shared_ptr<Private> _p;
};

}

#endif // MAX_IO_H
