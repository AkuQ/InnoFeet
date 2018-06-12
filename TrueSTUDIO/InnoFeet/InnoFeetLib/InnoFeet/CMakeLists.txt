cmake_minimum_required(VERSION 3.0)
project(InnoFeet C)

set(CMAKE_C_STANDARD 99)

include_directories(inc)

file(GLOB SOURCES src/*.c)
file(GLOB HEADERS inc/*.h)

add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC inc)