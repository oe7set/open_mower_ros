cmake_minimum_required(VERSION 3.0.2)
project(event_publisher)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        xbot_msgs)

find_package(Boost REQUIRED)

add_subdirectory(../json json EXCLUDE_FROM_ALL)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES event_publisher
        CATKIN_DEPENDS roscpp std_msgs xbot_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

add_library(event_publisher
        src/event_publisher.cpp)

target_link_libraries(event_publisher
        ${catkin_LIBRARIES}
        nlohmann_json::nlohmann_json)

add_dependencies(event_publisher
        ${catkin_EXPORTED_TARGETS}
        ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS event_publisher
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")
