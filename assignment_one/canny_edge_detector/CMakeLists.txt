cmake_minimum_required(VERSION 2.8)
project(canny_edge_detector)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++0x -Wall -Werror -g")
add_executable(canny_edge_detector src/application.cpp)
target_link_libraries(canny_edge_detector ${OpenCV_LIBS})
