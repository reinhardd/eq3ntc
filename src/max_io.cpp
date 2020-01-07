
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

void ncube::start()
{
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
                const std::string &data = std::get<std::string>(evt.second);
                process_data(data);
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

void ncube::process_data(const std::string &datain)
{
    if (datain.size() < 21)
        L_Err << "invalid msg: " << datain;
    L_Info << "got packet " << datain;

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
