
#include <set>
#include <iostream>
#include <filesystem>
#include <boost/bind.hpp>

#include "max_io_p.h"

#include "log.h"

namespace max_io {

enum struct event_e {
    Start,
    GotVersion,
    GotPacket,
    RxError,
    VersionTimeout,
    Restart,
};

namespace {
const char *delimiter = "\r\n";

enum struct cmds:uint8_t {
    PairPing                    = 0x00,
    PairPong                    = 0x01,
    Ack                         = 0x02,
    TimeInformation             = 0x03,
    ConfigWeekProfile           = 0x10,
    ConfigTemperatures          = 0x11,
    ConfigValue                 = 0x12,
    AddLinkPartner              = 0x20,
    RemoveLinkPartner           = 0x21,
    SetGroupId                  = 0x22,
    RemoveGroupId               = 0x23,
    ShutterContactState         = 0x30,
    SetTemperature              = 0x40,
    WallThermostatControl       = 0x42,
    SetComfortTemperature       = 0x43,
    SetEcoTemperature           = 0x44,
    PushButtonState             = 0x50,
    ThermostatState             = 0x60,
    WallThermostatState         = 0x70,
    SetDisplayActualTemperature = 0x82,
    Reset                       = 0xf0,
    WakeUp                      = 0xf1
};

std::string flags_to_string(uint8_t f)
{
    std::ostringstream xs;
    xs << std::hex << std::setfill('0') << std::setw(2) << uint16_t(f);
    return xs.str();
}

std::string devflags_to_string(uint8_t f)
{
    std::ostringstream xs;
    xs << std::hex << std::setfill('0') << std::setw(2) << uint16_t(f);
    xs << "|M:" << (f & 3)
       << ";DST:" << ((f & 8) == 0x08)
       << ";LNK:" << ((f & 0x10) == 0x10)
       << ";PLCK:" << ((f & 0x20) == 0x20)
       << ";RFERR:" << ((f & 0x40) == 0x40)
       << ";BATLOW:" << ((f & 0x80) == 0x80)
       << ";";
    return xs.str();
}

std::string dump(const std::string &s)
{
    std::ostringstream os;
    os << std::hex << std::setfill('0');
    bool first = true;
    for (uint8_t c: s)
    {
        if (!first)
            os << ' ';
        else
            first = false;

        os << std::setw(2) << uint16_t(c);
    }
    return os.str();
}

std::string build_cmd(const tx_data &txd, uint8_t counter)
{
    std::ostringstream builder;
    uint16_t len = 10 + txd.payload.size() / 2;
    builder << std::hex << std::setfill('0') << std::uppercase
            << "Zs" << std::setw(2) << len
            << std::setw(2) << uint16_t(counter)
            << std::setw(2) << uint16_t(txd.flag)
            << std::setw(2) << uint16_t(txd.msgtype)
            << std::setw(6) << txd.src
            << std::setw(6) << txd.dest
            << std::setw(2) << uint16_t(txd.gr_id)
            << txd.payload;
    return builder.str();
}

}

std::ostream &operator<<(std::ostream &s, const opmode &m)
{
    std::ostringstream xs;
    switch (m)
    {
        case opmode::automatic:
            xs << "auto";
            break;
        case opmode::manual:
            xs << "manual";
            break;
        case opmode::boost:
            xs << "boost";
            break;
        case opmode::temporary:
            xs << "temp";
            break;
    }
    s << xs.str();
    return s;
}

ncube::ncube(std::string port, const config &conf, callback cb)
    : ncube(port, conf, std::make_shared<ba::io_service>(), cb)
{
    L_Trace << "ncube on port " << port  << std::endl;
}

ncube::ncube(std::string port, const config &conf, std::shared_ptr<boost::asio::io_service> ios, callback cb)
    : _p(std::make_unique<Private>(ios, cb))
{
    L_Trace << "x ncube on port " << port  << std::endl;
    _p->port = port;
    _p->cdata = conf;
    start();
}

ncube::~ncube()
{
    _p->ios->stop();
    _p->thrd.join();
}

void ncube::mark_updated(rfaddr addr)
{
    if (is_thermostat(addr))
    {
        _p->thermostats.find(addr)->second.updated = std::chrono::steady_clock::now();
    }
}

void ncube::stop_overdue_poll()
{
    _p->overdueto.cancel();
}

void ncube::send_wakeup(rfaddr addr)
{
    L_Trace << __PRETTY_FUNCTION__;
}

void ncube::start_overdue_poll()
{
    L_Trace << __PRETTY_FUNCTION__;
    _p->overdueto.expires_from_now(std::chrono::seconds(10));
    _p->overdueto.async_wait([this](const boost::system::error_code &e){
        if (!e)
        {
            L_Info << "overdue timeout";
            rfaddr addr;
            bool have_one = next_overdue(addr);
            if (have_one)
            {
                L_Info << "overdue have one";
                send_wakeup(addr);
            }

            _p->overdueto.expires_from_now(std::chrono::seconds(have_one ? 20 : 2));
        }
        else
        {
            L_Info << "overdue cancel";
        }
    });
}

bool ncube::is_overdue(rfaddr addr)
{
    L_Trace << __PRETTY_FUNCTION__;
    auto &tstat_or_wall =
            is_thermostat(addr) ? _p->thermostats.find(addr)->second
                                : _p->wallthermostats.find(addr)->second;

    const auto &lastupd = tstat_or_wall.updated;

    std::chrono::steady_clock::time_point endt = lastupd + std::chrono::seconds(20);

    if (std::chrono::steady_clock::now() > endt)
    {
        auto diff = std::chrono::steady_clock::now() - endt;
        auto int_sec = std::chrono::duration_cast<std::chrono::seconds>(diff);

        L_Info << tstat_or_wall.name
               << " overdue for "
               << int_sec.count()
               << " seconds";
    }

    return false;
}

bool ncube::next_overdue(rfaddr &addr)
{
    L_Trace << __PRETTY_FUNCTION__;
#if 0
    unsigned maxsearches = _p->all_components.size();
    for (unsigned u = 0; u < maxsearches; ++u)
    {
        rfaddr a = _p->all_components[u];
        if (is_overdue(a))
        {
            addr = a;
            return true;
        }
    }
#endif
    return false;
}

void ncube::setup_workdata()
{
    for (const config::value_type &n: _p->cdata)
    {
        room_id roomno = n.first;
        const room_config &rc = n.second;
        for (auto cit = n.second.thermostats.begin(); cit != n.second.thermostats.end(); ++cit)
        {
            _p->thermostats.emplace(std::pair(cit->first, thermostat_state(cit->second)));
            _p->room_map[cit->first] = roomno;
            // _p->all_components.push_back(cit->first);
            L_Info << "added " << cit->second << " to " << roomno << ':' << rc.name;
        }

        if (rc.wallthermostat)
        {
            _p->wallthermostats.emplace(std::pair(rc.wallthermostat, thermostat_state("wt")));
            _p->room_map[n.second.wallthermostat] = n.first;
            // _p->all_components.push_back(rc.wallthermostat);
            L_Info << "added wt to " << roomno << ':' << rc.name;
        }
    }
}

bool ncube::is_thermostat(rfaddr rf) const
{
    return (_p->thermostats.find(rf) != _p->thermostats.end());
}

bool ncube::is_wallthermostat(rfaddr rf) const
{
    return (_p->wallthermostats.find(rf) != _p->wallthermostats.end());
}

void ncube::start()
{
    setup_workdata();
    _p->ios->post([this](){
        process_event(event_t(event_e::Start, std::monostate()));
    });
    _p->thrd = std::thread([this]() {
        L_Trace << "start asio " << _p->ios.get();
        // sync_setup_port();
        _p->ios->run();
        L_Trace << "asio terminated";
    });
}

bool ncube::open_port()
{
    _p->sio.open(_p->port);
    if (_p->sio.is_open())
    {
        _p->sio.set_option(ba::serial_port_base::baud_rate(38400));
        _p->sio.set_option(ba::serial_port_base::parity(ba::serial_port_base::parity::none));
        _p->sio.set_option(ba::serial_port_base::stop_bits(ba::serial_port_base::stop_bits::one));
        _p->sio.set_option(ba::serial_port_base::flow_control(ba::serial_port_base::flow_control::none));
        _p->sio.set_option(ba::serial_port_base::character_size(8));
        return true;
    }
    return false;
}

void ncube::write_sync(std::string data)
{
    L_Trace << "write sync: " << data;
    data.push_back('\r');
    data.push_back('\n');
    ba::write(_p->sio, boost::asio::buffer(data.c_str(), data.size()));
}

void ncube::sync_enable_moritz()
{
    write_sync("Zx");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    write_sync("X22");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    write_sync("Zr");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void ncube::send_async(std::string txd, write_callback sacb, unsigned max_tries)
{
#if 0
    Private::tx_unit txu(txd, sacb, max_tries);
    _p->txqueue.push_back(txu);
    if (_p->txqueue.size() == 1)
    {
        _p->txqueue.front().attempt = 1;
        start_async_write(_p->txqueue.front().txbuf,
                          //std::function<void (const boost::system::error_code &, std::size_t)> fn
            [this](const boost::system::error_code &err, std::size_t ) {
                if (err)
                {
                    auto first = _p->txqueue.front();
                    _p->txqueue.pop_front();
                    first.cb(false);
                }
        });
    }
#endif
}

void ncube::start_async_write(std::string txd, std::function<void (const boost::system::error_code &, std::size_t)> fn)
{
    L_Info << __FUNCTION__ << ": " << txd;
    txd.push_back('\r');
    txd.push_back('\n');
    _p->txbuf = txd;
    ba::async_write(_p->sio, ba::buffer(_p->txbuf.c_str(), _p->txbuf.size()), fn);
}

void ncube::start_packet_read()
{
    L_Trace << __FUNCTION__;
    ba::async_read_until(_p->sio, _p->rxbuf, delimiter,
                         boost::bind(&ncube::packet_read, this, ba::placeholders::error,
                                     ba::placeholders::bytes_transferred));
}

std::string ncube::get_received()
{
    std::string datain;
    {
        size_t nBufferSize = buffer_size(_p->rxbuf.data());
        ba::streambuf::const_buffers_type constBuffer = _p->rxbuf.data();

        std::ostringstream xs;
        std::copy(
             ba::buffers_begin(constBuffer),
             ba::buffers_begin(constBuffer) + nBufferSize,
             std::ostream_iterator<char>(xs)
        );
        datain = xs.str();
        _p->rxbuf.consume(nBufferSize);
    }
    return datain;
}

void ncube::packet_read(const boost::system::error_code& error_code,
                  std::size_t)
{
    L_Trace << __FUNCTION__;
    if (error_code)
        L_Err << "error " << error_code.message() << ": " << error_code.value();
    else
    {
        size_t nBufferSize = buffer_size(_p->rxbuf.data());
        ba::streambuf::const_buffers_type constBuffer = _p->rxbuf.data();

        std::string datain = get_received();
        L_Dbg << "received " << datain;

        if (datain.size())
        {
            switch (datain[0])
            {
                case 'V':
                    L_Info << "cul version " << datain;
                    process_event(event_t(event_e::GotVersion, datain));
                    break;
                case 'Z':
                    process_event(event_t(event_e::GotPacket, datain));
                    break;
                default:
                    L_Err << "unknown packet " << datain;
            }
        }
    }
    start_packet_read();
}

std::string ncube::devinfo(rfaddr a)
{
    if (!a)
        return "bcast";
    for (const auto n: _p->cdata)
    {
        if (n.second.wallthermostat == a)
            return n.second.name + ":WT";
        for (const auto t: n.second.thermostats)
        {
            if (t.first == a)
                return n.second.name + ":" + t.second;
        }
    }
    std::ostringstream xs;
    xs << std::setfill('0') << std::hex << std::setw(6) << a;
    return xs.str();
}

unsigned uvalue(const char *pD, unsigned len)
{
    unsigned rv = 0;
    std::string x;
    for (unsigned u = 0; u < len; ++u)
    {
        char c = pD[u];
        x += c;
        uint8_t val = 0;
        if (c >= 'A')
            val = static_cast<uint8_t>(c - 'A' + 10);
        else if (c <= '9')
            val = static_cast<uint8_t>(c - '0');
        rv = (rv << 4) + val;
    }
    // val.erase(0,len);
    return rv;
}

void ncube::process_event(const max_io::event_t &evt)
{
    L_Trace << __FUNCTION__ << " " << static_cast<int>(evt.first);
    switch (evt.first)
    {
        case event_e::Start:
            _p->cstate = comstate::setupPort;
            if (open_port())
            {
                start_packet_read();
                start_async_write("V", [](const boost::system::error_code &e, std::size_t){
                    if (!e)
                        L_Trace << "Version cmd write done";
                    else
                        L_Err << "Version cmd write failed";
                });

                _p->versionto.expires_from_now(std::chrono::seconds(10));
                _p->versionto.async_wait([this](const boost::system::error_code &e){
                    if (!e)
                    {
                        L_Err << "Version cmd timeout";
                        process_event(event_t(event_e::VersionTimeout,std::monostate()));
                    }
                    else
                        L_Trace << "Version cmd canceled";
                });
            }
            else
                L_Err << "unable to open port";
            break;
        case event_e::GotVersion:
            _p->versionto.cancel();
            if (_p->cstate == comstate::setupPort)
            {
                _p->cstate = comstate::receiving;
                sync_enable_moritz();
                L_Info << "start input processing";
            }
            break;
        case event_e::GotPacket:
            {
                std::string data = std::get<std::string>(evt.second);
                data.pop_back(); // remove cr
                data.pop_back(); // remove lf
                process_data(std::move(data));
            }
            break;
        case event_e::VersionTimeout:
            _p->sio.cancel();
            ba::write(_p->sio, ba::buffer("V\r\n", 3));
            _p->versionto.expires_from_now(std::chrono::seconds(3));
            _p->versionto.async_wait([this](const boost::system::error_code &e){
                if (!e)
                    process_event(event_t(event_e::VersionTimeout,std::monostate()));
            });
            break;
    }
}

std::string temp_end2_string(uint32_t endt)
{
    unsigned month = ((endt >> 20) & 0xe) + (endt & 0x8000 ? 1 : 0);
    unsigned day = (endt >> 15) & 0x1f;
    unsigned year = 2000 +((endt >> 8) & 0x3f);
    unsigned hour = (endt & 0x3f) / 2;
    unsigned min = (endt & 0x3f) % 2 ? 30 : 0;
    std::ostringstream aux;
    aux << day << '.' << month << '.' << year
        << ' ' << hour << ':' << min;
    return aux.str();
}

std::string temp_end_set_vacation(uint32_t endt)
{
#if 0
             765432107654321076543210
    81930d = 100000011001001100001101 = 1.9.19 6:30
             ||||||||||||||||||||||||
             ||||||||||||||||||++++++-----------------------Time *2 (001100 = 13 / 2 = 6.5 = 6:30)
             ||||||||||||||||++-----------------------------???
             ||||||||||++++++-------------------------------year (010011 = 19)
             |||||||||+-------------------------------------???
             |||+++++|--------------------------------------Day (00001 = 1)
             +++-----+--------------------------------------month (1001 = 9 = September)
#endif
    unsigned month = ((endt >> 21) << 1) + (endt & 0x8000 ? 1 : 0);
    unsigned day = (endt >> 16) & 0x1f;
    unsigned year = 2000 +((endt >> 8) & 0x3f);
    unsigned hour = (endt & 0x3f) / 2;
    unsigned min = (endt & 0x3f) % 2 ? 30 : 0;
    std::ostringstream aux;
    aux << day << '.' << month << '.' << year
        << std::setfill('0')
        << ' ' << std::setw(2) << hour << ':' << std::setw(2) << min;
    return aux.str();
}

void ncube::process_data(std::string &&datain)
{
    std::size_t dsz = datain.size();
    if (dsz < 21)
    {
        L_Err << "invalid msg: " << datain;
        return;
    }

    // uint8_t possibly_rssi = uvalue(datain.end()-2, 2);
    // L_Info << "got packet sz " << datain.size() << " : " << datain;
    auto get_input_info = [&datain]() {
        std::ostringstream xs;
        xs << "data[" << datain.size() << ": " << datain << "]";
        return xs.str();
    };

    // 12345678901234567890123456789012345
    // Z0F 00  04   60  1AF6DD 000000 00 18 00 0C 00 CD F8
    //     cnt flag mt  src    dest   payload           rssi
    // 0f -> 36
    // 0f * 2 => 30 +

    // datain.erase(datain.size()-2);



    const char *pData = datain.c_str();
    unsigned len = uvalue(pData+1, 2);
    // L_Info << "len " << len << " sz " << datain.size();
    int8_t rssi = 0;
    bool with_rssi = (dsz == (len * 2 + 5));
    if (with_rssi)
    {
        rssi = static_cast<int8_t>(uvalue(pData+dsz-2, 2));
        datain.pop_back();
        datain.pop_back();
        dsz = datain.size();
    }

    auto getdata = [&pData, &dsz](uint8_t ofs, unsigned len) {
        if (len >= dsz)
        {
            L_Err << "invalid access to ofs " << ofs << " max " << dsz;
            return 0u;
        }
        return uvalue(pData + ofs, len);
    };

    unsigned cnt = getdata(3, 2); // uvalue(pData+3, 2);
    uint8_t flag = getdata(5, 2);
    unsigned mt = getdata(7, 2);

    unsigned src = getdata(9, 6);
    unsigned dst = getdata(15, 6);
    std::string payload = std::string(pData + 21, pData + 21 + ((len - 9) * 2));

    unsigned payloadsize = payload.size() / 2;

    // Z112F02021AF6DD1BEEFD00 011A00091F9422F6 len 17 szlen 34  16 = 34 - 17 - 1 => 21 + len*2
    //    12345678901234567890 12345678901234

    switch (static_cast<cmds>(mt))
    {
        case cmds::Ack:
            if (is_thermostat(src))
            {
                uint8_t roomid = getdata(21, 2);
                uint8_t state = getdata(23, 2);
                uint8_t flags = getdata(25, 2);
                uint8_t valve = getdata(27, 2);
                uint8_t desired = getdata(29, 2);
                std::ostringstream aux;
                if (datain.size() > 33)
                {
                    uint32_t endt = getdata(31, 6);
                    aux << temp_end2_string(endt);
                }
                L_Info << get_input_info()
                       << " -> { Ack rsp state:" << uint16_t(state)
                       << " mode:" << int(flags & 3)
                       << " valve:" << uint16_t(valve)
                       << " desired:" << float(desired/2.0)
                       << " flags:" << devflags_to_string(flags)
                       << aux.str()
                       << "}";

                update_thermostat_mode_valve_desired(src, static_cast<opmode>(flags & 3), valve, float(desired/2.0));

                // check if we have a pending transmit waiting
            }
            else
                L_Err << "src " << std::hex << std::setw(6) << src << "not a thermostat";
            break;
        case cmds::ThermostatState:
            if (is_thermostat(src))
            {
                uint8_t roomid = static_cast<uint8_t>(getdata(21, 2));
                uint8_t flags = static_cast<uint8_t>(getdata(23, 2));
                uint8_t valve = static_cast<uint8_t>(getdata(25, 2));
                uint8_t desired = static_cast<uint8_t>(getdata(27, 2));
                uint8_t until1 = static_cast<uint8_t>(getdata(29, 2));
                uint8_t until2 = static_cast<uint8_t>(getdata(31, 2));

                opmode mode = static_cast<opmode>(flags & 3);

                float measured;

                bool withmeasured = (mode == opmode::automatic || mode == opmode::manual);
                std::ostringstream xs;
                if (withmeasured)
                {
                    uint16_t mraw = until1 * 0x100 + until2;
                    measured = (mraw & 0x1ff) / 10.0;
                    xs << " measured:" << measured;
                }

                L_Info << get_input_info()
                       << " -> { TState:" << uint16_t(roomid)
                       << " flags:" << std::hex << uint16_t(flags) << std::dec
                       << " mode:" << unsigned(mode)
                       << " valve:" << uint16_t(valve)
                       << " desired:" << float(desired/2.0)
                       << xs.str() << " }";
                if (withmeasured)
                    update_thermostat_mode_valve_desired_measured(src, static_cast<opmode>(flags & 3), valve, float(desired/2.0), measured);
                else
                    update_thermostat_mode_valve_desired(src, static_cast<opmode>(flags & 3), valve, float(desired/2.0));

            }
            break;
        case cmds::SetTemperature:
            {
                uint8_t room = static_cast<uint8_t>(getdata(21, 2));
                uint8_t cb = static_cast<uint8_t>(getdata(23, 2));
                unsigned mode = (cb >> 6);
                float desired = (cb & 0x3f) / 2.0;

                std::ostringstream xs;
                if (mode == 2) // vacation
                {
                    uint32_t temp_end = static_cast<uint32_t>(getdata(25, 6));
                    // until decoding not right here
                    xs << " vacation until " << temp_end_set_vacation(temp_end);
                }
                else if (mode == 1)
                {
                    switch(cb)
                    {
                        case 0x41:
                            xs << " manual eco";
                            break;
                        case 0x42:
                            xs << " manual comfort";
                            break;
                        case 0x43:
                            xs << " manual window open";
                            break;
                        default:
                            xs << " manual " << desired << "°C";
                            break;
                    }
                }

                L_Info << get_input_info() << " -> { set room: " << uint16_t(room)
                       << " mode: " << unsigned(mode)
                       << " desired: " << desired
                       << xs.str()
                       << "}";
            }
            break;
        case cmds::WallThermostatState:
            if (is_wallthermostat(src))
            {
                L_Info << "wtstate";
            }
            break;
        case cmds::WallThermostatControl:
            {
                if (payloadsize != 3)
                    L_Err << "invalid payload size expected 3 got " << payloadsize;
                uint8_t misc = static_cast<uint8_t>(getdata(21, 2));
                uint8_t desired_raw = static_cast<uint8_t>(getdata(23, 2));
                uint8_t measured_raw = static_cast<uint8_t>(getdata(25, 2));
                float desired = (desired_raw & 0x7F) / 2.0;
                float measured = ((uint16_t(desired_raw & 0x80) << 1) + measured_raw) / 10.0 ;

                L_Info << get_input_info() << " -> { wt-ctrl desired:" << desired << " measured:" << measured << "}";

                update_wallthermostat_desired_measured(src, desired, measured);

            }
            break;
        default:
            L_Err << get_input_info() << " {unknown cmd 0x" << std::hex << std::setw(2) << std::setfill('0')
                  << mt << std::dec << "}";
            break;
    }

    std::ostringstream rssiinfo;
    if (with_rssi)
        rssiinfo << ") rssi(" << int16_t(rssi);

    L_Info << "MT " << std::hex << mt << ' '
           << " cnt(" << std::dec << uint16_t(cnt)
           << ") s(" << devinfo(src)
           << ") d(" << devinfo(dst)
           << ") flags(" << flags_to_string(flag)
           << rssiinfo.str()
           << ") pl(" << payload << ")";

#if 0


                    1 2 3 4 5 6 7 8 9 0 1
      got packet Z0B7D054017068D1AF5AE0624

      MT 40  s(Cube:WT) d(Flur:Flur) pl 06


      got packet Z0E7D02021AF5AE17068D0001181024

      MT 2  s(Flur:Flur) d(Cube:WT) pl 00011

  got packet Z0E DF 02 02 1A98FC 1B6592 0001180628

  MT 2  s(Wohnzimmer:Erker links) d(Wohnzimmer:WT) pl 000118


#endif

}

thermostat_state::thermostat_state(std::string name)
    : name(name)
    , updated(std::chrono::steady_clock::now())
    , counter(1)
{}

bool thermostat_state::operator!=(const thermostat_state &ts)
{
    return (
        (desired != ts.desired) ||
        (measured != ts.measured) ||
        (mode != ts.mode) ||
        (dst_active != ts.dst_active) ||
        (panel_locked != ts.panel_locked) ||
        (rf_err != ts.rf_err) ||
        (link_valid != ts.link_valid) ||
        (battery_ok != ts.battery_ok) ||        
        (name != ts.name));
}

bool ncube::room_from_rfaddr(rfaddr src, unsigned &roomnr)
{
    unsigned roomno = 0;
    auto cit = _p->room_map.find(src);
    if (cit != _p->room_map.end())
    {
        roomno = cit->second;
    }
    roomnr = roomno;
    return (roomnr != 0);
};

void ncube::update_thermostat_mode_valve_desired_measured(rfaddr src, opmode m, uint16_t valve, float desired, float measured)
{
    unsigned roomno;
    if (!room_from_rfaddr(src, roomno))    // if (roomno == 0)
    {
        L_Err << "no room found for rfaddr 0x" << std::hex << std::setw(6)
              << std::setfill('0') << src;
        return;
    }
    L_Trace << "update thermostat for room " << roomno;
    auto tmapit = _p->thermostats.find(src);
    if (tmapit == _p->thermostats.end())
    {
        L_Err << "unable to find thermostat with rfaddr "
              << std::hex << std::setw(6)
              << std::setfill('0') << src;
        return;
    }
    thermostat_state &ts = tmapit->second;
    bool smvd = set_mode_valve_desired(ts, m, valve, desired);
    bool smd = set_measured_desired(ts, desired, measured);
    if (smvd || smd)
        emit_update();

}

void ncube::update_thermostat_mode_valve_desired(rfaddr src, opmode m, uint16_t valve, float desired)
{
    unsigned roomno;
    if (!room_from_rfaddr(src, roomno))    // if (roomno == 0)
    {
        L_Err << "no room found for rfaddr 0x" << std::hex << std::setw(6)
              << std::setfill('0') << src;
        return;
    }
    L_Trace << "update thermostat for room " << roomno;
    auto tmapit = _p->thermostats.find(src);
    if (tmapit == _p->thermostats.end())
    {
        L_Err << "unable to find thermostat with rfaddr "
              << std::hex << std::setw(6)
              << std::setfill('0') << src;
        return;
    }
    thermostat_state &ts = tmapit->second;
    if (set_mode_valve_desired(ts, m, valve, desired))
        emit_update();
}

void ncube::update_wallthermostat_desired_measured(rfaddr src, float desired, float measured)
{
    unsigned roomno;
    if (!room_from_rfaddr(src, roomno))    // if (roomno == 0)
    {
        L_Err << "no room found for rfaddr 0x" << std::hex << std::setw(6)
              << std::setfill('0') << src;
        return;
    }
    auto tmapit = _p->wallthermostats.find(src);
    if (tmapit == _p->wallthermostats.end())
    {
        L_Err << "unable to find wallthermostat with rfaddr "
              << std::hex << std::setw(6)
              << std::setfill('0') << src;
        return;
    }
    thermostat_state &ts = tmapit->second;

    if (set_measured_desired(ts, desired, measured))
        emit_update();
}

bool ncube::set_mode_desired(thermostat_state &ts, opmode mode, float desired)
{
    bool update = ((ts.mode != mode) || (ts.desired != desired));
    if (update)
    {
        ts.desired = desired;
        ts.mode = mode;
    }
    return update;
}

bool ncube::set_measured_desired(thermostat_state &ts, float desired, float measured)
{
    bool update = ((ts.desired != desired) || (ts.measured != measured));
    if (update)
    {
        ts.desired = desired;
        ts.measured = measured;
    }
    return update;
}

bool ncube::set_mode_valve_desired(thermostat_state &ts, opmode m, uint16_t valve, float desired)
{
    bool update = ((ts.valve != valve) ||
                   (ts.desired != desired) ||
                   (ts.mode != m));
    if (update)
    {
        ts.valve = valve;
        ts.desired = desired;
        ts.mode = m;
    }
    return update;
}


void ncube::emit_update()
{
    L_Trace << __PRETTY_FUNCTION__;

    std::map<unsigned, room_state> rms;
    for (const auto &n: _p->thermostats)
    {
        rfaddr addr = n.first;
        const thermostat_state &ts = n.second;
        unsigned roomno = _p->room_map[addr];
        const room_config &rc = _p->cdata[roomno];

        std::string tname;
        auto ctstat = rc.thermostats.find(n.first);

        if (ctstat != rc.thermostats.end())
            tname = ctstat->second;

        rms[roomno].thermostats.push_back(ts);
        rms[roomno].thermostats.back().name = tname;

        rms[roomno].name = rc.name;
    }

    for (const auto &n: _p->wallthermostats)
    {
        rfaddr addr = n.first;
        const thermostat_state &ts = n.second;
        unsigned roomno = _p->room_map[addr];
        const room_config &rc = _p->cdata[roomno];

        rms[roomno].desired = n.second.desired;
        rms[roomno].measured = n.second.measured;
    }


    std::shared_ptr<system_state> ssp = std::make_shared<system_state>();

    for (auto &x: rms)
    {
        // calc valve average
        room_state &rs = x.second;

        unsigned valvesum = 0;
        unsigned valvecnt = 0;

        float measured = 0;
        unsigned measuredcnt = 0;

        float desired = 0;
        unsigned desiredcnt = 0;

        if (rs.thermostats.size())
        {
            for (const auto &t: rs.thermostats)
            {
                if (t.valve)
                {
                    valvesum += t.valve;
                    valvecnt++;
                }
                if (t.measured != 0.0)
                {
                    measured += t.measured;
                    measuredcnt++;
                }
                if (t.desired)
                {
                    desired += t.desired;
                    desiredcnt++;
                }
            }
            if (valvecnt)
                rs.valve = valvesum / valvecnt;
            if (!x.second.wallthermostat)
            {
                if (measuredcnt)
                    rs.measured = measured / measuredcnt;
                if (desiredcnt)
                    rs.desired = desired / desiredcnt;
            }
            ssp->roomstates.emplace_back(rs);
        }
    }
    _p->host_cb(ssp);
}

bool ncube::get_room_id(rfaddr addr, uint8_t &rid) const
{
    auto room_it = _p->room_map.find(addr);
    if (room_it != _p->room_map.end())
    {
        rid = room_it->second;
        return true;
    }
    return false;
}

void ncube::_wakeup(rfaddr addr)
{
    L_Info << __FUNCTION__;

    std::map<rfaddr, thermostat_state>::iterator it;

    if (is_thermostat(addr))
    {
        it = _p->thermostats.find(addr);
    }
    else if (is_wallthermostat(addr))
    {
        it = _p->wallthermostats.find(addr);
    }
    else
    {
        L_Err << "dev " << std::hex << std::setw(6) << std::setfill('0') << addr
              << " is neither thermostat nor wallthermostat";
        return;
    }

    uint8_t roomid = 0;
    if (!get_room_id(addr, roomid))
    {
        L_Err << "unable to find roomid for dev " << std::hex << std::setw(6) << std::setfill('0') << addr;
        return;
    }
    auto room_it = _p->room_map.find(addr);
    if (room_it != _p->room_map.end())
        roomid = room_it->second;

    uint8_t counter = it->second.counter++;

    std::string tosend = build_cmd(tx_data(uint8_t(0x05), uint8_t(0xf1), _p->cdata[0].wallthermostat, addr, roomid, std::string("3F")), counter);
    start_async_write(tosend,[](const boost::system::error_code &err, std::size_t sz){
        L_Info << "wakeup send done " << err.message() << ':' << sz;
    });
}

void ncube::start_job(job j, job_callback cb)
{
    L_Trace << __FUNCTION__ << ": "  << static_cast<int>(j.first);
    _p->ios->post(std::bind(&ncube::_run_job, this, j, cb));
}

uint8_t ncube::get_inc_counter(rfaddr addr)
{
    std::map<rfaddr, thermostat_state>::iterator it;
    if (is_thermostat(addr))
    {
        it = _p->thermostats.find(addr);
    }
    else if (is_wallthermostat(addr))
    {
        it = _p->wallthermostats.find(addr);
    }
    else
        return 0;

    uint8_t counter = it->second.counter++;
    if (counter == 0)   // skip zero -> signals error
        counter = it->second.counter++;
    return counter;
}

bool ncube::start_send(const tx_data &txd, uint8_t counter)
{
    L_Trace << __PRETTY_FUNCTION__ << " to " << txd.dest;
    std::string tosend = build_cmd(txd, counter);
    start_async_write(tosend,[](const boost::system::error_code &err, std::size_t sz){
        L_Info << "send done " << err.message() << ':' << sz;
    });
    return true;
}

bool ncube::add_send_job(tx_data &&txd, job_callback cb)
{
    rfaddr dest = txd.dest;
    L_Trace << __PRETTY_FUNCTION__ << " to " << dest;
    if (_p->tx_states.find(dest) == _p->tx_states.end())
    {
        L_Trace << "create tx state\n";
        _p->tx_states[dest] = std::make_unique<dev_tx_status>(_p->ios.get());
    }


    auto &dev = _p->tx_states[dest];

    if (dev->current_tx)
    {
        L_Trace << "add to queue";
        dev->waiting_for_send.push_back(tx_unit(txd, cb));
    }
    else
    {
        L_Trace << "send direct ";
        uint8_t counter = get_inc_counter(dest);
        if (!counter)
        {
            return false;
        }
        dev->waiting_on = counter;
        dev->current_tx = tx_unit(txd, cb);
        tx_unit &txu = *(dev->current_tx);
        start_send(txu.txd, dev->waiting_on);
        dev->rxto.expires_from_now(std::chrono::seconds(3));

        dev->rxto.async_wait([this, dest, counter](const boost::system::error_code &e){
            L_Info << "rxto expired for " << std::hex << dest << " err: " << e.message();
            send_job_response(dest, counter, !e);
        });

    }
    return false;
}

void ncube::send_job_response(rfaddr dest, uint8_t counter, bool res)
{
    L_Info << __FUNCTION__ << " addr " << std::hex << dest << " cnt " << counter << " res " << res;

    auto n = _p->tx_states.find(dest);
    if (n == _p->tx_states.end())
    {
        L_Err << "not found any device state for " << std::hex << dest;
        return;
    }
    auto &dev = n->second;
    if (dev->current_tx)
    {
        dev->current_tx->cb(res);
        dev->current_tx.reset();
    }
    else
    {
        L_Err << "lost state";
    }
}

void ncube::_run_job(job j, job_callback cb)
{
    L_Info << __FUNCTION__ << " job: " << static_cast<int>(j.first);
    switch (j.first)
    {
        case job_type::test_wakeup:
            {
                rfaddr todev = std::get<rfaddr>(j.second);
                uint8_t roomid = 0;
                if (!get_room_id(todev, roomid))
                {
                    L_Err << "unable to get room id";
                    cb(false);
                    return;
                }
                tx_data senddata(0x05, 0xf1, _p->cdata[0].wallthermostat, todev, roomid, std::string("3F"));
                if (!add_send_job(std::move(senddata), cb))
                    cb(false);
                else
                {
                    L_Info << "tx initiated";
                }

                // _wakeup(std::get<rfaddr>(j.second));
            }
            break;
    }
}
}
