#cmake需求版本
cmake_minimum_required(VERSION 2.8)
#项目名称
project(icra2017_102)
#添加标准库支持
set(CMAKE_CXX_FLAGS "-std=c++11")
#寻找opencv库
find_package(OpenCV REQUIRED)
#添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(/home/tdt/three_alarm/include)
#可执行文件
add_executable(icra main.cpp serial.cpp)
add_executable(video video.cpp serial.cpp)
#链接Opencv库
target_link_libraries(icra ${OpenCV_LIBS} -lpthread)
target_link_libraries(video ${OpenCV_LIBS} -lpthread )
