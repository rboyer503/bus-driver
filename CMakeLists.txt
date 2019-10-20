cmake_minimum_required (VERSION 2.8)
 
project (bus-driver)

find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})
add_executable (bus-driver Socket.cpp SocketMgr.cpp VideoCaptureMgr.cpp BusMgr.cpp ControlMgr.cpp LaneTransform.cpp bus-driver.cpp)
target_link_libraries (bus-driver ${OpenCV_LIBS} -lboost_thread -lboost_system -lboost_date_time -lv4l2 -pthread -lpigpio -lrt)
