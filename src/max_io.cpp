
#include <set>
#include <iostream>
#include <filesystem>
#include <boost/bind.hpp>

#include "max_io.h"
#include "log.h"


namespace ba = boost::asio;

namespace max_io {

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


}

enum struct comstate {
    setupPort,
    receiving
};

using tstamp = std::chrono::time_point<std::chrono::system_clock>;
using tstamped_temp = std::pair<float, tstamp>;

struct thermostat_actual_values
{
    tstamped_temp set;
    tstamped_temp current;
    unsigned valve; // 0 - 100

    void set_set(float f)
    {
        set.first = f;
        set.second = std::chrono::system_clock::now();
    }
    void set_current(float f)
    {
        current.first = f;
        current.second = std::chrono::system_clock::now();
    }
    void set_valve(unsigned v)
    {
        valve = v;
    }
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

    std::string txbuf;

    comstate cstate;

    // config rw
    config cdata;
    // workdata
    std::thread thrd;    
    std::map<rfaddr, thermostat_actual_values> thermostats;
    std::set<rfaddr> wallthermostats;

    Private(std::shared_ptr<ba::io_service> io)
        : ios(io)
        , sio(*(ios.get()))
        , work(*(ios.get()))
        , versionto(*(ios.get()))
    {
        L_Dbg << "ios " << ios.get();
    }
};

ncube::ncube(std::string port, const config &conf)
{
    L_Trace << "ncube on port " << port  << std::endl;
    _p = std::make_shared<Private>(std::make_shared<ba::io_service>());
    _p->cdata = conf;
    _p->port = port;
    start();
}

ncube::ncube(std::string port, const config &conf, std::shared_ptr<boost::asio::io_service> ios)
{
    _p = std::make_shared<Private>(ios);
    _p->port = port;
    _p->cdata = conf;    
    start();
}

void ncube::setup_workdata()
{
    for (const config::value_type &n: _p->cdata)
    {
        for (auto cit = n.second.thermostats.begin(); cit != n.second.thermostats.end(); ++cit)
            _p->thermostats[cit->first] = thermostat_actual_values();

        if (n.second.wallthermostat)
            _p->wallthermostats.insert(n.second.wallthermostat);
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


void ncube::sync_enable_moritz()
{
    ba::write(_p->sio, boost::asio::buffer("Zx\r\n", 4));     // disable
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ba::write(_p->sio, boost::asio::buffer("X22\r\n", 4));    // hex with RSSI
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ba::write(_p->sio, ba::buffer("Zr\r\n", 4));
}

void ncube::sync_setup_port()
{
    if (!open_port())
        throw std::runtime_error("port open failed");
    ba::write(_p->sio, boost::asio::buffer("Zx\r\n", 4));     // disable
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ba::write(_p->sio, boost::asio::buffer("X22\r\n", 4));    // hex with RSSI
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ba::write(_p->sio, ba::buffer("Zr\r\n", 4));
}

void ncube::start_async_write(const std::string &txd, std::function<void (const boost::system::error_code &, std::size_t)> fn)
{
    _p->txbuf = txd;
    ba::async_write(_p->sio, ba::buffer(_p->txbuf.c_str(), _p->txbuf.size()), fn);
}

void ncube::setup_port(const boost::system::error_code& error_code, std::size_t, unsigned state)
{
    L_Trace << __FUNCTION__ << " state " << state;
    if (state == 0)
    {
        if (open_port())
        {
            start_async_write("V\r\n",
                            [](const boost::system::error_code& error_code, std::size_t)
            {
                if (error_code)
                    L_Err << "error on write " << error_code.message();
                else
                    L_Info << "write done";
            });
            ba::async_read_until(_p->sio, _p->rxbuf, delimiter,
                                 boost::bind(&ncube::setup_port, this,
                                             ba::placeholders::error,
                                             ba::placeholders::bytes_transferred, 1));
        }
        else
            L_Fatal << "unable to open port " << _p->port;
    }
    else if (state == 1)
    {
        std::string input = get_received();
        L_Info << "received " << input;

        start_async_write("Zx\r\n",
                        [](const boost::system::error_code& error_code, std::size_t)
        {
            if (error_code)
                L_Err << "error on write " << error_code.message();
            else
                L_Info << "write done";
        });
        ba::async_read_until(_p->sio, _p->rxbuf, delimiter,
                             boost::bind(&ncube::setup_port, this,
                                         ba::placeholders::error,
                                         ba::placeholders::bytes_transferred, 2));
    }
}

void ncube::initialize_port()
{
    L_Trace << __FUNCTION__;

    start_async_write("V\r\n",
                    [](const boost::system::error_code& error_code, std::size_t)
    {
        if (error_code)
            L_Err << "write failed " << error_code.message();
        else
            L_Trace << "data written";
    });

    L_Trace << "start read";
    start_packet_read();
}

void ncube::evaluate_version(std::string &&data)
{
    L_Trace << "CUL Version:" << data;
    // enable_moritz();
}

void ncube::evaluate_packet(std::string &&data)
{
    L_Trace << "packet data :" << data;

    start_packet_read();
}



void ncube::start_packet_read()
{
    L_Trace << "start read\n";
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
        L_Trace << "received " << datain;

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
    L_Trace << "process_event " << static_cast<int>(evt.first);
    switch (evt.first)
    {
        case event_e::Start:
            _p->cstate = comstate::setupPort;
            if (open_port())
            {
                start_packet_read();
                ba::write(_p->sio, ba::buffer("V\r\n", 4));
                _p->versionto.expires_from_now(std::chrono::seconds(3));
                _p->versionto.async_wait([this](const boost::system::error_code &e){
                    if (!e)
                        process_event(event_t(event_e::VersionTimeout,std::monostate()));
                });
            }
            break;
        case event_e::GotVersion:
            if (_p->cstate == comstate::setupPort)
            {
                _p->versionto.cancel();
                _p->cstate = comstate::receiving;
                sync_enable_moritz();
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
            ba::write(_p->sio, ba::buffer("V\r\n", 4));
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

void ncube::process_data(std::string &&datain)
{
    if (datain.size() < 21)
        L_Err << "invalid msg: " << datain;
    L_Info << "got packet sz " << datain.size() << " : " << datain;

    // 12345678901234567890123456789012345
    // Z0F 00  04   60  1AF6DD 000000 00 18 00 0C 00 CD F8
    //     cnt flag mt  src    dest   payload           rssi
    // 0f -> 36
    // 0f * 2 => 30 +

    // datain.erase(datain.size()-2);
    const char *pData = datain.c_str();
    unsigned len = uvalue(pData+1, 2);
    unsigned cnt = uvalue(pData+3, 2);
    unsigned flag = uvalue(pData+5, 2);
    unsigned mt = uvalue(pData+7, 2);

    unsigned src = uvalue(pData+9, 6);
    unsigned dst = uvalue(pData+15, 6);
    std::string payload = std::string(pData+21, pData+21+len*2-8);

    switch (static_cast<cmds>(mt))
    {
        case cmds::Ack:
            if (is_thermostat(src))
            {
                uint8_t roomid = uvalue(pData+21, 2);
                uint8_t state = uvalue(pData+23, 2);
                uint8_t flags = uvalue(pData+25, 2);
                uint8_t valve = uvalue(pData+27, 2);
                uint8_t desired = uvalue(pData+29, 2);
                std::ostringstream aux;
                if (datain.size() > 33)
                {
                    uint32_t endt = uvalue(pData+31, 6);
                    aux << temp_end2_string(endt);
#if 0
                    // 1000 0000 0000 0000 = 0x8000
                    unsigned month = ((endt >> 20) & 0xe) + (endt & 0x8000 ? 1 : 0);
                    unsigned day = (endt >> 15) & 0x1f;
                    unsigned year = 2000 +((endt >> 8) & 0x3f);
                    unsigned hour = (endt & 0x3f) / 2;
                    unsigned min = (endt & 0x3f) % 2 ? 30 : 0;
                    aux << " temp end " << day << '.' << month << '.' << year
                        << ' ' << hour << ':' << min;
#endif
                }
                L_Info << "Ack rsp state:" << uint16_t(state)
                       << " mode:" << int(flags & 3)
                       << " valve:" << uint16_t(valve)
                       << " desired:" << float(desired/2.0)
                       << aux.str();

            }
            else
                L_Err << "src " << src << "not a thermostat";
            break;
        case cmds::ThermostatState:
            if (is_thermostat(src))
            {
                uint8_t roomid = static_cast<uint8_t>(uvalue(pData+21, 2));
                uint8_t flags = static_cast<uint8_t>(uvalue(pData+23, 2));
                uint8_t valve = static_cast<uint8_t>(uvalue(pData+25, 2));
                uint8_t desired = static_cast<uint8_t>(uvalue(pData+27, 2));
                uint8_t until1 = static_cast<uint8_t>(uvalue(pData+29, 2));
                uint8_t until2 = static_cast<uint8_t>(uvalue(pData+31, 2));

                L_Info << "TState :" << uint16_t(roomid)
                       << " flags:" << std::hex << uint16_t(flags) << std::dec
                       << " valve:" << uint16_t(valve)
                       << " desired:" << float(desired/2.0);

            }
            break;
        case cmds::SetTemperature:
            {
#if 0
                [2020-01-22 21:05:27.326364] [0x00007fe58e4a9700] [info]    got packet Z0B73054017068D1A9CA701285B
                [2020-01-22 21:05:27.326403] [0x00007fe58e4a9700] [info]    set mode and temp: 0 0.5°C
                [2020-01-22 21:05:27.326448] [0x00007fe58e4a9700] [info]    MT 40  s(Cube:WT) d(Wohnzimmer:Erker rechts) pl 01285B

                  01285B
                  01 room
                  28 mode + temp    0x28 & 0x7f => 0x28 => 40 / 2 => 20 °C
                  5B
                  5B => 91

                  // with vacation to 31.1.2020 18:00
                  [2020-01-22 22:09:11.998688] [0x00007f2a4ad78700] [trace]   process_event 2
                  [2020-01-22 22:09:11.998715] [0x00007f2a4ad78700] [info]    got packet sz 33 : Z0E8E054017068D1AF58705891F942459
                  [2020-01-22 22:09:11.998767] [0x00007f2a4ad78700] [info]    set room: 5 mode: 2 temp: 4.5vacation temp 15.5 temp end  temp end 8.8.2036 12:30
                  [2020-01-22 22:09:11.998822] [0x00007f2a4ad78700] [info]    MT 40  s(Cube:WT) d(ELW Schlafzimmer:ELW Schlafzimmer) pl 05891F942459

                  Z0E8E0540 17068D 1AF587 05891F942459
                  05    room
                  89    cb mode(89 >> 6 => 2 =>vac)  desired(89 & 7f => 9 9 / 2 => 4.5)
                  1F942459

                  Mode 1 Manual
                  [2020-01-22 22:19:40.912969] [0x00007f8dbad42700] [info]    got packet sz 27 : Z0B8F05 40 17068D 1AF587 05495C
                  [2020-01-22 22:19:40.913001] [0x00007f8dbad42700] [info]    set room: 5 mode: 1
                  [2020-01-22 22:19:40.913067] [0x00007f8dbad42700] [info]    MT 40  s(Cube:WT) d(ELW Schlafzimmer:ELW Schlafzimmer) pl 05495C

                  05  room
                  49  cd Mode(49 >> 6 => 1 =>manual) desired(49&3f => 9 /2.0 => 4.5)
                  5C


#endif
                uint8_t room = static_cast<uint8_t>(uvalue(pData+21, 2));
                uint8_t cb = static_cast<uint8_t>(uvalue(pData+23, 2));
                unsigned mode = (cb >> 6);
                float desired = (cb & 0x3f) / 2.0;

                std::ostringstream xs;
                if (mode == 2) // vacation
                {
                    uint32_t temp_end = static_cast<uint32_t>(uvalue(pData+27, 6));
                    // until decoding not right here
                    xs << " vacation " << desired << "°C until " << temp_end2_string(temp_end);
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

                L_Info << "set room: " << uint16_t(room)
                       << " mode: " << unsigned(mode)
                       << xs.str();
            }
            break;
        case cmds::WallThermostatState:
            if (is_wallthermostat(src))
            {

            }
            break;
        case cmds::WallThermostatControl:
            {
                uint8_t misc = static_cast<uint8_t>(uvalue(pData+21, 2));
                uint8_t desired_raw = static_cast<uint8_t>(uvalue(pData+23, 2));
                uint8_t measured_raw = static_cast<uint8_t>(uvalue(pData+25, 2));
#if 0
                wt ctrl desired:0 measured:4.1
                MT 42  s(Wohnzimmer:WT) d(Wohnzimmer:Spitzerker links) pl 0029D31B
                0029D31B
                  desired_raw 0x29 b00101001
                  desired 0x29 & 0x27 => 0x29 => 41 / 2.0 => 20,1



#endif

                float desired = (desired_raw & 0x7F) / 2.0;
                float measured = ((uint16_t(desired_raw & 0x80) << 1) + measured_raw) / 10.0 ;

                L_Info << "wt ctrl desired:" << desired << " measured:" << measured;

            }
            break;
        default:
            L_Err << "unknown cmd 0x" << std::hex << std::setw(2) << std::setfill('0') << mt << std::dec;
            break;
    }


    L_Info << "MT " << std::hex << mt << ' '
           << " s(" << devinfo(src)
           << ") d(" << devinfo(dst)
           << ") pl " << payload
           << std::endl;

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

}
