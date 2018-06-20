cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)
project(InnoFeetTests)


# Fetch googletest files during configuration:
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/tests/googletest-download" )
execute_process(COMMAND "${CMAKE_COMMAND}" --build .
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/tests/googletest-download" )

# Add googletest to build:
add_subdirectory(
        "${CMAKE_BINARY_DIR}/tests/googletest-src"
        "${CMAKE_BINARY_DIR}/tests/googletest-build"
)

# Link googletest to project:
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

add_executable(runQueueTests queue_test.cpp)
target_link_libraries(runQueueTests gmock_main)
target_link_libraries(runQueueTests InnoFeet)
