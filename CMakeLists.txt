cmake_minimum_required(VERSION 2.8.3)
project(gps_node)

# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS 
			roscpp 
			std_msgs
            geometry_msgs
			tf
)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include src ${catkin_INCLUDE_DIRS})
add_executable(gps_node src/gps_node.cpp src/test_gps.cpp
               src/Uart.h src/Uart.cpp src/serial/serial.h src/serial/serial.cc
		       src/UbloxGPS.h src/UbloxGPS.cpp src/Socket.h src/Socket.cpp
               src/serial/impl/unix.cc src/serial/impl/unix.h
               src/serial/impl/list_ports/list_ports_linux.cc)
target_link_libraries(gps_node ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-fpermissive -std=c++11")
