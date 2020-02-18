#ifndef MAX_IO_P_H
#define MAX_IO_P_H

#include <memory>
#include <chrono>
#include <deque>

#include <boost/asio.hpp>
#include "max_io.h"
#include "log.h"

namespace ba = boost::asio;

namespace max_io {

enum struct comstate {
    setupPort,
    receiving
};

struct response_status {

    tstamp updated { std::chrono::steady_clock::now() };

    enum struct oss {
        initial,
        valid,
        pollstarted,
        slowpoll
    } state { oss::valid };

    long next_to_ms()
    {
        std::chrono::steady_clock::time_point endt;
        switch (state)
        {
            case oss::initial:
                endt = updated + std::chrono::seconds(120);
                break;
            case oss::valid:
                endt = updated + std::chrono::seconds(300);
                break;
            case oss::pollstarted:
                endt = updated + std::chrono::minutes(5);
                break;
            case oss::slowpoll:
                endt = updated + std::chrono::hours(1);
                break;
        }

        auto diff = endt - std::chrono::steady_clock::now();
        auto int_msec = std::chrono::duration_cast<std::chrono::milliseconds>(diff);
        return int_msec.count();
    }
    void set_state(oss s)
    {
        state = s;
        updated = std::chrono::steady_clock::now();
    }
    void mark_valid()
    {
        set_state(oss::valid);
    }
    void mark_unresponsive()
    {
        set_state(oss::pollstarted);
    }

};

struct tx_data {
    uint8_t flag;
    uint8_t msgtype;
    rfaddr src;
    rfaddr dest;
    uint8_t gr_id;  // group/room id
    std::string payload;
    tx_data(uint8_t flg, uint8_t mt, rfaddr s, rfaddr d, uint8_t gid, const std::string &pl)
        : flag(flg), msgtype(mt), src(s), dest(d), gr_id(gid), payload(pl)
    {}
};

struct tx_unit
{
    tx_data txd;
    unsigned maxtries;
    unsigned attempt;
    job_callback cb;
    tx_unit(tx_data tx, job_callback wcb, unsigned mtries = 3)
        : txd(tx)
        , maxtries(mtries)
        , attempt(0)
        , cb(wcb)
    {}
};

struct dev_tx_status
{
    uint8_t waiting_on;
    ba::steady_timer rxto;

    dev_tx_status(ba::io_service *ios)
        : rxto(*ios)
    {
    }

    ~dev_tx_status()
    {
        rxto.cancel();
    }
    // std::deque<tx_data> waiting_for_send;
    std::deque<tx_unit> waiting_for_send;

    std::optional<tx_unit> current_tx;
};

struct ncube::Private
{
    std::shared_ptr<ba::io_service> ios;
    ba::serial_port sio;
    ba::io_service::work work;
    std::string port;
    ba::streambuf rxbuf;

    ba::steady_timer versionto;

    ba::steady_timer overdueto;


    /** the transmit queue.
     * the first one is the one that is actually processed,
     **/
    // std::deque<tx_unit> txqueue;

    using addr_txstate_map_t = std::map<rfaddr, std::unique_ptr<dev_tx_status> >;
    addr_txstate_map_t  tx_states;


    std::string txbuf;

    comstate cstate;

    // config rw
    config cdata;
    // workdata
    std::thread thrd;

    // next 2 map's were filled by setup_workdata
    std::map<rfaddr, thermostat_state> thermostats;
    std::map<rfaddr, thermostat_state> wallthermostats;

    using overdue_ctrl_t = std::map< rfaddr, response_status>;
    overdue_ctrl_t all_components;

    unsigned last_event_cnt { 0 };
    overdue_ctrl_t::iterator overdue_last_checked;

    std::map<rfaddr, room_id> room_map;

    callback host_cb;            // callback to our host



    Private(std::shared_ptr<ba::io_service> io, callback &ncb)
        : ios(io)
        , sio(*(ios.get()))
        , work(*(ios.get()))
        , versionto(*(ios.get()))
        , overdueto(*ios)
        , host_cb(ncb)
    {
        L_Dbg << "ios " << ios.get();
    }
};
}

#endif // MAX_IO_P_H
