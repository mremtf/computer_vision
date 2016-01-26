cmake_minimum_required(VERSION 2.8)
project(gaussian_pyramid)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++0x -Wall -Werror -g")
add_executable(gp src/application.cpp src/gaussian_pyramid.cpp)
target_link_libraries(gp ${OpenCV_LIBS}) 


