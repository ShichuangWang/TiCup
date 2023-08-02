cmake_minimum_required(VERSION 3.21)
project(TiCup)
set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)

add_executable(TiCup main.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(TiCup ${OpenCV_LIBS})
