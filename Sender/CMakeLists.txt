cmake_minimum_required(VERSION 3.16)

message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(PROJECT_VER "${GIT_COMMIT_HASH}")

message(STATUS "Firmware version set to: ${PROJECT_VER}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(intercom)
