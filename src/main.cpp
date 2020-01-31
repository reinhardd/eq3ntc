
#include <iostream>
#include <iomanip>
#include <thread>

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
    L_Trace << "cube started";

    while (true)
        std::this_thread::sleep_for(std::chrono::seconds(1000));

    L_Trace << "cube terminate";

    return 0;
}


void init()
{
    boost::log::core::get()->set_filter
    (
        boost::log::trivial::severity >= boost::log::trivial::trace
    );
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
