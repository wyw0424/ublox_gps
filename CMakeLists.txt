CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(ublox_gps)

SET(CMAKE_DEBUG_POSTFIX "d")
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR};${CMAKE_MODULE_PATH}")

MACRO(NEW_EXECUTABLE EXECUTABLE_NAME)

    ADD_EXECUTABLE(${EXECUTABLE_NAME} ${EXECUTABLE_FILES})

    SET_TARGET_PROPERTIES(${EXECUTABLE_NAME} PROPERTIES DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
    SET_TARGET_PROPERTIES(${EXECUTABLE_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}/lib"
                                                        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}/lib"
                                                        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}/lib"
                                                        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}/lib"
                                                        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}/bin"
                                                        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}/bin")

    INSTALL(TARGETS ${EXECUTABLE_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
                                       LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                                       ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

    TARGET_INCLUDE_DIRECTORIES(${EXECUTABLE_NAME} PUBLIC ${EXTERNAL_INCLUDES})
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${EXTERNAL_LIBRARIES})

ENDMACRO(NEW_EXECUTABLE)

IF(NOT WIN32)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti -std=c++11")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-register")
ENDIF(NOT WIN32)

INCLUDE_DIRECTORIES(.)


find_package(Eigen3 REQUIRED)
# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS 
			roscpp 
			std_msgs
            geometry_msgs
            sensor_msgs
			tf
)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(ublox_gps_ros_node src/ublox_gps_ros_node.cpp Uart.cpp UbloxGPS.cpp libserial/SerialPort.cpp libserial/SerialStream.cpp libserial/SerialStreamBuf.cpp)

target_link_libraries(ublox_gps_ros_node ${catkin_LIBRARIES})
