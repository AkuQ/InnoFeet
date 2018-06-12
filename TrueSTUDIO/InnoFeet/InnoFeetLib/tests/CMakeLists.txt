cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)

project(Tests)

add_subdirectory(googletest-master)
#include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
target_include_directories(
        gmock_main SYSTEM BEFORE INTERFACE
        ${gtest_SOURCE_DIR}/include
        ${gmock_SOURCE_DIR}/include
)


add_executable(runMorseTests morse_test.cpp)
target_link_libraries(runMorseTests gmock_main)
target_link_libraries(runMorseTests InnoFeet)

add_executable(runI2C_Tests i2c_test.cpp)
target_link_libraries(runI2C_Tests gmock_main)
target_link_libraries(runI2C_Tests InnoFeet)

add_executable(runByteConversionTests byte_conversion_test.cpp)
target_link_libraries(runByteConversionTests gmock_main)
target_link_libraries(runByteConversionTests InnoFeet)

add_executable(runSensorTests sensor_test.cpp)
target_link_libraries(runSensorTests gmock_main)
target_link_libraries(runSensorTests InnoFeet)
