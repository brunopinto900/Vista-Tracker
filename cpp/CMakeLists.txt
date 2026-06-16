cmake_minimum_required(VERSION 3.10)

project(VistaTracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(
    tracker

    src/main.cpp
    src/config/ConfigLoader.cpp
    src/control/PIDController.cpp
    src/estimation/PerfectEstimator.cpp
    src/mapping/FakeESDFMap.cpp
    src/perception/GroundTruthPerception.cpp
    src/planning/SimplePlanner.cpp
    src/sim_impl/KinematicSim.cpp
    src/sim_impl/WaypointFollower.cpp
)

target_link_libraries(
    tracker
    yaml-cpp
)
