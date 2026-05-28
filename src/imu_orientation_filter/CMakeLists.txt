cmake_minimum_required(VERSION 3.0.2)
project(imu_orientation_filter)

add_compile_options(-Wall -Wextra -Wpedantic -O2)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
    xbot_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs xbot_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/imu_orientation_filter_node.cpp
    src/madgwick_ahrs.cpp
)

# xbot_msgs generates AbsolutePose.h at build time; without this dependency
# imu_orientation_filter can be compiled before the headers exist.
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
