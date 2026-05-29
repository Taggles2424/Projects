cmake_minimum_required(VERSION 3.5)

project(blue_pill LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3)  # for OpenCV
find_package(OpenCV REQUIRED)

#------ uncomment if OpenCV is needed
include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_executable(blue_pill main.cpp)
target_link_libraries(blue_pill ${OpenCV_LIBRARIES}
)
