cmake_minimum_required(VERSION 3.22)
project(RRT_star)

set(CMAKE_CXX_STANDARD 17)

add_executable(RRT_star
        RRT_star.cpp
        RRT_star.h
        Environment.cpp
        Environment.h
        WebSocketServer.cpp
        WebSocketServer.h
        main.cpp
        stoppableThread.cpp
        stoppableThread.h
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(RRT_star ${OpenCV_LIBS})

#set(octomap_DIR "/opt/ros/humble/share/octomap")
set(octomap_DIR "/Users/ivancist/Documents/CLionProjects/lib/octomap-1.10.0/lib/cmake/octomap")
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(RRT_star ${OCTOMAP_LIBRARIES})

set(dynamicEDT3D_DIR "/Users/ivancist/Documents/CLionProjects/lib/octomap-1.10.0/lib/cmake/dynamicEDT3D")
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
target_link_libraries(RRT_star ${DYNAMICEDT3D_LIBRARIES})

set(websocketpp_DIR "/Users/ivancist/Documents/CLionProjects/lib/websocketpp/build/wspp/lib/cmake/websocketpp")
find_package(websocketpp REQUIRED)
include_directories(${WEBSOCKETPP_INCLUDE_DIR})
target_link_libraries(RRT_star ${WEBSOCKETPP_LIBRARIES})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(nlohmann_json_DIR "/Users/ivancist/Documents/CLionProjects/lib/json-develop/build")
find_package(nlohmann_json REQUIRED)
include_directories(${NLOHMANN_JSON_INCLUDE_DIR})
target_link_libraries(RRT_star nlohmann_json::nlohmann_json)

#find_package(Open3D REQUIRED)
#include_directories(${Open3D_INCLUDE_DIRS})
#target_link_libraries(RRT_star ${Open3D_LIBRARIES})


