cmake_minimum_required(VERSION 3.15)
project(robot-simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON)

file(DOWNLOAD
        https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.42.1/CPM.cmake
        ${CMAKE_BINARY_DIR}/cmake/CPM.cmake)
include(${CMAKE_BINARY_DIR}/cmake/CPM.cmake)

set(CMAKE_MESSAGE_LOG_LEVEL NOTICE)

CPMAddPackage(
        NAME mujoco
        GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
        GIT_TAG 3.4.0
        QUIET
)

set(CMAKE_MESSAGE_LOG_LEVEL STATUS)

add_subdirectory(example)
