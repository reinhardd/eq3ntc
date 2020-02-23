
#include <iostream>
#include <iomanip>
#include <thread>
#include <boost/algorithm/string.hpp>

#include <yaml-cpp/yaml.h>

#include "max_io.h"
#include "log.h"

void init();

void read_config(max_io::config &cnf);
void safe_config(const max_io::config &cnf);

static const char *yaml_file = "test.yaml";

int main(int argc, char *argv[])
{
    init();

    max_io::config cconfig;
    read_config(cconfig);

    {
    L_Trace << "start cube";
    max_io::ncube mycube(argv[1], cconfig,[](std::shared_ptr<max_io::system_state> psysp){
        std::ostringstream xs;

        for (const auto n: psysp->roomstates)
        {
            xs << "\n" << n.name << std::fixed
               << "\nd:" << std::setw(4) << std::setprecision(1) << n.desired
               << "°C m:" << std::setw(4) << std::setprecision(1) << n.measured
               << "°C v:" << std::setw(3) << n.valve << "%";
               // << " tstats[";
            for (const auto m: n.thermostats)
            {
                xs << "\n" << std::string(20, ' ') << std::setw(16) << m.name
                   << "(d:" << std::setw(4) << m.desired
                   << "°C m:" << std::setw(4) << m.measured
                   << "°C v:" << std::setw(3) << m.valve
                   << "% mode:" << m.mode
                   << ")";
                if (!m.battery_ok || !m.link_valid || m.rf_err)
                {
                    xs << " Err: ";
                    if (!m.battery_ok)
                        xs << "bat-low;";
                    if (!m.link_valid)
                        xs << "link-err;";
                    if (m.rf_err)
                        xs << "rf-err;";
                }
            }
        }
        L_Trace << xs.str();

    });
    L_Trace << "cube started -- enter cmd loop";

    bool exit = false;
    while (!exit)
    {
        // std::this_thread::sleep_for(std::chrono::seconds(1000));
        std::string input;
        std::getline(std::cin, input);
        std::cout << "cmd " << input << std::endl;
        boost::trim(input);
        if (input == "quit")
            exit = true;
        else if (input.substr(0,3) == "wup")
        {
            input.erase(0,3);
            boost::trim(input);
            max_io::rfaddr rfaddr = 0;
#if 1
            rfaddr = mycube.get_addr(input);
#else
            std::istringstream ix(input);            
            ix >> std::hex >> rfaddr;
#endif
            if (rfaddr == 0)
                std::cerr << "invalid device name " << input << std::endl;
            else
            {
                std::cout << "main: addr " << std::hex << rfaddr << std::endl;
                using job_type = max_io::ncube::job_type;
                using job_data = max_io::ncube::job_data;
                mycube.start_job(job_data(job_type::test_wakeup, rfaddr), [rfaddr](bool res){
                    L_Info << "main: wup job finished for " << std::hex << rfaddr
                           << " res: " << std::boolalpha << res;
                });
            }
        }

    }

    L_Trace << "cube terminate";
    }
    L_Trace << "cube terminated";
    return 0;
}


void init()
{
    namespace logging = boost::log;
    namespace src = boost::log::sources;
    namespace expr = boost::log::expressions;
    namespace keywords = boost::log::keywords;
#if 0
    logging::add_file_log
    (
        keywords::file_name = "sample_%N.log",
        // This makes the sink to write log records that look like this:
        // 1: <normal> A normal severity message
        // 2: <error> An error severity message
        keywords::format =
        (
            expr::stream
                << expr::attr< unsigned int >("LineID")
                << ": <" << logging::trivial::severity
                << "> " << expr::smessage
        )
    );
#endif
    logging::core::get()->set_filter
    (
        boost::log::trivial::severity >= boost::log::trivial::trace
    );    
    L_Info << "start logging\n";
}

void read_config(max_io::config &cnf)
{
    max_io::config tmp;
    YAML::Node config = YAML::LoadFile(yaml_file);
    // YAML::Node rooms = config["rooms"];
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
        std::string roomname = it->first.as<std::string>();
        std::cout << "room " << it->first.as<std::string>() << std::endl;
        YAML::Node room = it->second;
        max_io::rfaddr_name thermostats;
        uint8_t rid = room["id"].as<uint16_t>();
        if (room["thermostats"])
        {
            YAML::Node tstats = room["thermostats"];
            // std::cout << "rit type " << tstats.Type() << std::endl;
            for (YAML::const_iterator tit = tstats.begin(); tit != tstats.end(); ++tit)
            {
                // std::cout << tit->first << ':' << tit->second.as<uint32_t>() << std::endl;
                thermostats[tit->second.as<uint32_t>()] = tit->first.as<std::string>();
            }
        }
        max_io::room_config nr(rid, roomname);
        nr.thermostats.swap(thermostats);
        if (room["wallthermostat"])
        {
            nr.wallthermostat = room["wallthermostat"].as<uint32_t>();
        }
        tmp[rid] = nr;
    }
    cnf.swap(tmp);
}

void safe_config(const max_io::config &cnf)
{

}
