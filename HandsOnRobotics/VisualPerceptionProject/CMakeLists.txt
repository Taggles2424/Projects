cmake_minimum_required(VERSION 3.5)
project(visual_perception LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable high-level compiler optimizations
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Compiler specific optimizations (GCC/Clang)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O3 -march=native -ffast-math)
endif()


# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Compile library containing our SLAM modules
file(GLOB SLAM_SOURCES "src/*.cpp")
if(SLAM_SOURCES)
    add_library(slam_lib STATIC ${SLAM_SOURCES})
    target_link_libraries(slam_lib ${OpenCV_LIBRARIES})
else()
    # Dummy library to prevent cmake errors if src is empty
    add_library(slam_lib INTERFACE)
endif()

# Compile the main executable
add_executable(visual_perception main.cpp)
target_link_libraries(visual_perception slam_lib ${OpenCV_LIBRARIES})

# Compile intermediate test executables
file(GLOB TEST_SOURCES "tests/*.cpp")
foreach(test_source ${TEST_SOURCES})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(${test_name} ${test_source})
    target_link_libraries(${test_name} slam_lib ${OpenCV_LIBRARIES})
endforeach()
