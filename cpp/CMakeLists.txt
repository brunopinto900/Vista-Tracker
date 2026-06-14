cmake_minimum_required(VERSION 3.10)

project(target_tracking_pid)

set(CMAKE_CXX_STANDARD 17)

find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(
    tracker

    src/main.cpp

    src/controllers/PID.cpp
    src/controllers/PIDController.cpp

    src/simulation/Simulator.cpp

    src/config/ConfigLoader.cpp
)

target_link_libraries(
    tracker
    yaml-cpp
)