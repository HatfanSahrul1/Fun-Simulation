cmake_minimum_required(VERSION 3.8)

project(seat C CXX)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add support for threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Find YAML-CPP library
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(${YAML_CPP_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

# Add executable and link libraries
add_executable(mcl mcl.cpp)
target_link_libraries(mcl
    ${OpenCV_LIBS}
    yaml-cpp        # Corrected YAML-CPP library link
    Threads::Threads
)
