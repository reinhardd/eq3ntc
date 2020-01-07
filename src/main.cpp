
#include <iostream>

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
    max_io::ncube mycube(argv[1], cconfig);
    L_Trace << "cube started";

    std::this_thread::sleep_for(std::chrono::seconds(1000));

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
        max_io::room nr(rid, roomname);
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
