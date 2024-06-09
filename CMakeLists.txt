cmake_minimum_required(VERSION 3.22)
project(oppi)

set(CMAKE_CXX_STANDARD 17)

add_executable(oppi
        main.cpp
        RRT_star.cpp
        RRT_star.h
        Environment.cpp
        Environment.h
        WebSocketServer.cpp
        WebSocketServer.h
        stoppableThread.cpp
        stoppableThread.h
)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(octomap_DIR "/Users/ivancist/Documents/CLionProjects/lib/octomap-1.10.0/lib/cmake/octomap")
    set(dynamicEDT3D_DIR "/Users/ivancist/Documents/CLionProjects/lib/octomap-1.10.0/lib/cmake/dynamicEDT3D")
    set(websocketpp_DIR "/Users/ivancist/Documents/CLionProjects/lib/websocketpp/build/wspp/lib/cmake/websocketpp")
    set(nlohmann_json_DIR "/Users/ivancist/Documents/CLionProjects/lib/json-develop/build")
else ()
    set(octomap_DIR "/opt/ros/humble/share/octomap")
    set(dynamicEDT3D_DIR "/opt/ros/humble/share/dynamicEDT3D")
endif()

include_directories(/usr/include/websocketpp /usr/include/nlohmann)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(oppi ${OpenCV_LIBS})


find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(oppi ${OCTOMAP_LIBRARIES})


find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
target_link_libraries(oppi dynamicedt3d)

find_package(websocketpp REQUIRED)
include_directories(${WEBSOCKETPP_INCLUDE_DIR})
target_link_libraries(oppi ${WEBSOCKETPP_LIBRARIES})

find_package(nlohmann_json REQUIRED)
include_directories(${NLOHMANN_JSON_INCLUDE_DIR})
target_link_libraries(oppi nlohmann_json::nlohmann_json)

#find_package(ompl REQUIRED)
#include_directories(${OMPL_INCLUDE_DIRS})
#target_link_libraries(oppi ${OMPL_LIBRARIES})