cmake_minimum_required (VERSION 2.8.11)
if (APPLE)
	set(CMAKE_C_COMPILER "/opt/homebrew/bin/gcc-11")
	set(CMAKE_CXX_COMPILER "/opt/homebrew/bin/g++-11")
endif()
project (missileModel)
set(CMAKE_BUILD_TYPE Debug)
include_directories(
	./
)
file(GLOB SOURCES "./*.cpp")
add_executable(missileModel ${SOURCES})