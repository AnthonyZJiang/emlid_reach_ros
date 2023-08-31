cmake_minimum_required(VERSION 3.1)
project(emlid_reach_ros VERSION 1.1)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    serial
    std_msgs
    nmea_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs serial nmea_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(sources
    src/nmea/nmea_parser.cpp
    src/nmea/conversion.cpp
    src/nmea/nmea_sentence.cpp
)

add_library(${PROJECT_NAME} ${sources})

add_executable(reach_serial scripts/reach_serial.cpp)
target_link_libraries(reach_serial ${catkin_LIBRARIES} ${PROJECT_NAME})