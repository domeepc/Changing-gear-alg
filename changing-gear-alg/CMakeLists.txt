cmake_minimum_required(VERSION 3.10)

project(changing_gear_alg)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

file(GLOB SOURCES "src/*.cpp")

add_executable(changing_gear_alg ${SOURCES})