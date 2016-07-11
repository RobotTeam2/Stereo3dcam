cmake_minimum_required(VERSION 2.8)
project( camera )
find_package( OpenCV REQUIRED )
find_package( libuvc REQUIRED)
add_executable( camera camera.cpp )
target_link_libraries( camera ${OpenCV_LIBS} ${libuvc_LIBRARIES} usb-1.0 pthread m )

add_executable( camera.mt camera.mt.cpp )
target_link_libraries( camera.mt ${OpenCV_LIBS} ${libuvc_LIBRARIES} usb-1.0 pthread m )

