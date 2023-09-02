cmake_minimum_required(VERSION 3.1)
project(emlid_reach_ros VERSION 0.1)

# Set minimum C++ standard to C++11
if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    serial
    sensor_msgs
    geometry_msgs
    nmea_msgs
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    src/nmea/nmea_parser.cpp
    src/nmea/nmea_sentence.cpp
    src/nmea/conversion.cpp
    src/reach_driver/reach_driver.cpp
    src/reach_driver/reach_serial_driver.cpp
    src/sat_nav.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(reach_node src/reach_node.cpp)
target_link_libraries(reach_node ${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
