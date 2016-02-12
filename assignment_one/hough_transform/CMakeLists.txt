cmake_minimum_required(VERSION 2.8)
project(hough_transform)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++0x -Wall -Werror -g")
add_executable(hough_transform src/application.cpp src/hough_transform.cpp)
target_link_libraries(hough_transform ${OpenCV_LIBS} m)

#add_executable(gp_test src/g_pyramid.cpp)
#target_link_libraries(gp_test ${OpenCV_LIBS})



