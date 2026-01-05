cmake_minimum_required(VERSION 3.14)
project(rrt_first_benchmark VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release by default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Find packages
find_package(OpenCV REQUIRED)
find_package(ompl REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system)

# RRT First-Solution Benchmark
add_executable(rrt_first_benchmark
    src/rrt_first_solution_main.cpp
    src/rrt_first_solution.cpp
)

target_include_directories(rrt_first_benchmark PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(rrt_first_benchmark
    ${OpenCV_LIBS}
    ${OMPL_LIBRARIES}
    yaml-cpp
    ${Boost_LIBRARIES}
    pthread
)

# Print build info
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV: ${OpenCV_VERSION}")
message(STATUS "OMPL: ${OMPL_INCLUDE_DIRS}")