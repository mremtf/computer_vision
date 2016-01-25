cmake_minimum_required(VERSION 2.8)
project(gaussian_pyramid)
# Locate GTest
find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++0x -Wall -Werror -g")
# Link runTests with what we want to test and the GTest and pthread library
add_executable(${PROJECT}_test test/tests.cpp)
target_link_libraries(${PROJECT}_test ${GTEST_LIBRARIES} pthread)

add_executable(${PROJECT} src/gaussian_pryamid.cpp)

enable_testing()
add_test(NAME    ${PROJECT}_test 
         COMMAND ${PROJECT}_test)


