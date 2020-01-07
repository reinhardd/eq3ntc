cmake_minimum_required(VERSION 3.8)
project(NTCube VERSION 0.0.1 LANGUAGES CXX)

find_package(Boost REQUIRED COMPONENTS log;thread;chrono;program_options)

find_package(yaml-cpp REQUIRED)

add_executable(ntcube
    src/main.cpp
    src/max_io.cpp
)

set_target_properties(ntcube PROPERTIES
    CXX_STANDARD 17
    INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS};${YAML_CPP_INCLUDE_DIR}"
    LINK_LIBRARIES "${Boost_LIBRARIES};${YAML_CPP_LIBRARIES}"
)
