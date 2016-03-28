cmake_minimum_required(VERSION 2.8)
project(assignment_four)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++0x -Wall -g")
add_executable(thresholding src/thresholding.cpp)
target_link_libraries(thresholding ${OpenCV_LIBS})


add_executable(histogram src/histogram.cpp)
target_link_libraries(histogram ${OpenCV_LIBS})

add_executable(kmeans_segmentation src/kmeans_segmentation.cpp)
target_link_libraries(kmeans_segmentation ${OpenCV_LIBS})
