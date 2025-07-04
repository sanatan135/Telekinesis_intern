cmake_minimum_required(VERSION 3.10)
project(H1_WBC_Controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Create a simple executable first
add_executable(h1_wbc_demo
    src/robot_model.cpp
    src/main.cpp
)

# Link libraries
target_link_libraries(h1_wbc_demo
    Eigen3::Eigen
)

# Compiler flags
target_compile_options(h1_wbc_demo PRIVATE
    -Wall -Wextra -O2
)

message(STATUS "Building H1 WBC Controller")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
