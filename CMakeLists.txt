# CMakeLists.txt

# Minimum required version of CMake
cmake_minimum_required(VERSION 3.22)

# Project name
project(MSP_DIO)

add_subdirectory(MSP_Interface)

set(CMAKE_C_COMPILER ${CMAKE_SOURCE_DIR}/msp430-gcc-9.3.1.11_linux64/bin/msp430-elf-gcc)

# Sources
set(SRC_DIR src)

set(SOURCES
    ${SRC_DIR}/Dio_Driver.c
)

# Include directories
include_directories(include)

# Compiler flags
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "-mmcu=msp430g2553 -O2 -g -Wall -c")

# Add executable
add_executable(Dio_Driver.o ${SOURCES})

# Link libraries
target_link_libraries(Dio_Driver.o MSP_INTERFACE)
