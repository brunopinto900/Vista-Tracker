cmake_minimum_required(VERSION 3.14)

project(VistaTracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)

include_directories(include)

# ── Main executable ───────────────────────────────────────────────────────────

add_executable(tracker
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

target_link_libraries(tracker yaml-cpp)

# ── Tests ─────────────────────────────────────────────────────────────────────

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(vista_tests
    tests/test_config_loader.cpp
    tests/test_waypoint_follower.cpp
    src/config/ConfigLoader.cpp
    src/sim_impl/WaypointFollower.cpp
)

target_include_directories(vista_tests PRIVATE include)
target_compile_definitions(vista_tests PRIVATE
    TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures"
)
target_link_libraries(vista_tests
    GTest::gtest_main
    yaml-cpp
)

include(GoogleTest)
gtest_discover_tests(vista_tests)
