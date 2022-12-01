cmake_minimum_required (VERSION 2.8.11)
project (ROCKET6G)
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-w)
include_directories(
	./
)
file(GLOB SOURCES "./*.cpp")
add_executable(ROCKET6G ${SOURCES})