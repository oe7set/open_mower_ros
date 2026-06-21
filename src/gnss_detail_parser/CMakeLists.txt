cmake_minimum_required(VERSION 3.0.2)
project(gnss_detail_parser)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_srvs
        xbot_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp std_srvs xbot_msgs
)

include_directories(
        include
        src
        src/minmea
        ${catkin_INCLUDE_DIRS}
)

# minmea is C; timegm() needs a POSIX feature macro on glibc.
add_definitions(-D_DEFAULT_SOURCE)

add_executable(gnss_detail_parser_node
        src/gnss_detail_parser_node.cpp
        src/parser/nmea_parser.cpp
        src/parser/ubx_parser.cpp
        src/minmea/minmea.c
)

add_dependencies(gnss_detail_parser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gnss_detail_parser_node
        ${catkin_LIBRARIES}
)

install(TARGETS gnss_detail_parser_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
