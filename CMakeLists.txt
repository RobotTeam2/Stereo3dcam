cmake_minimum_required(VERSION 2.8)
project( camera )
find_package( OpenCV REQUIRED )
find_package( libuvc REQUIRED)
include_directories( /usr/include/libusb-1.0 )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
add_executable( camera.mt camera.mt.cpp r2_chassis.cpp)
target_link_libraries( camera.mt ${OpenCV_LIBS} ${libuvc_LIBRARIES} usb-1.0 pthread m )

