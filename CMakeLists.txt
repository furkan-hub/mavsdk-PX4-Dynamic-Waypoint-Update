cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(takeoff_and_land)

# Include directories
include_directories(${CMAKE_SOURCE_DIR})

# Add executable with source files
add_executable(takeoff_and_land
    takeoff_and_land.cpp
    coordinates.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(takeoff_and_land
    MAVSDK::mavsdk
)

if(NOT MSVC)
    target_compile_options(takeoff_and_land PRIVATE -Wall -Wextra)
else()
    target_compile_options(takeoff_and_land PRIVATE -W2)
endif()
