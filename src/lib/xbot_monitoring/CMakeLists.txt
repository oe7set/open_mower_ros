cmake_minimum_required(VERSION 3.11)
project(xbot_monitoring)

## Compile as C++17
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
        xbot_msgs
        mower_msgs
        roscpp
        roslib
        geometry_msgs
        nav_msgs
        std_msgs
        visualization_msgs
        grid_map_core
        grid_map_ros
        grid_map_msgs
        rosgraph_msgs
        xbot_rpc
        dynamic_reconfigure
        )


find_package(
        Boost
)


## System dependencies are found with CMake's conventions
find_package(PahoMqttCpp REQUIRED)
set(PahoMqttCpp_LIBRARIES PahoMqttCpp::paho-mqttpp3)

# yaml-cpp for the meta.config.* YAML bridge.
find_package(yaml-cpp REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES mower_comms
        #  CATKIN_DEPENDS mower_msgs roscpp serial
        #  DEPENDS system_lib
        DEPENDS PahoMqttCpp

)

add_subdirectory(../json json EXCLUDE_FROM_ALL)

###########
## Build ##
###########

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(xbot_monitoring
        src/xbot_monitoring.cpp
        src/config_io.cpp
        src/yaml_io.cpp
        src/events_io.cpp
        src/sensors_history.cpp)


add_dependencies(xbot_monitoring ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(xbot_monitoring ${catkin_LIBRARIES} ${PahoMqttCpp_LIBRARIES} nlohmann_json::nlohmann_json yaml-cpp)

add_executable(xbot_sensor_example
        src/xbot_sensor_example.cpp)


add_dependencies(xbot_sensor_example ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(xbot_sensor_example ${catkin_LIBRARIES})
add_executable(heatmap_generator
        src/heatmap_generator.cpp)


add_dependencies(heatmap_generator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(heatmap_generator ${catkin_LIBRARIES})

# Optional standalone test for config_io. Build with:
#   catkin build xbot_monitoring --cmake-args -DBUILD_CONFIG_IO_TEST=ON
# Run from the binary directory and pass the schema path as argv[1].
option(BUILD_CONFIG_IO_TEST "Build the standalone config_io smoke test" OFF)
if(BUILD_CONFIG_IO_TEST)
    add_executable(config_io_test src/config_io.cpp src/config_io_test.cpp)
    target_link_libraries(config_io_test nlohmann_json::nlohmann_json)
endif()
