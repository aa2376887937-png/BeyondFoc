cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME BeyondFOC)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# C++ 标准（用户代码使用了 constexpr / override / 默认成员初始化等）
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Framework：生产者/消费者基础设施（header-only）
    # Framework/spsc_ring.h
    # Framework/latest.h
    # Framework/subject.h
    # Framework/encoder_frame.h

    # Platform：底层外设封装，只依赖 HAL/CMSIS/FreeRTOS
    Platform/RC_adc.cpp
    Platform/RC_analog.cpp
    Platform/RC_encoder.cpp
    Platform/RC_io.cpp
    Platform/RC_prof.cpp
    Platform/RC_pwm.cpp
    Platform/RC_task.cpp
    Platform/RC_tim.cpp
    Platform/RC_timer.cpp
    Platform/RC_uart.cpp

    # Algorithm：纯算法，不依赖 HAL
    Algorithm/FOC_control.cpp
    Algorithm/FOC_current.cpp
    Algorithm/FOC_math.cpp
    Algorithm/FOC_openloop.cpp
    Algorithm/FOC_pi.cpp
    Algorithm/FOC_position.cpp
    Algorithm/MT6701.cpp

    # App：应用层，负责装配、任务、命令、遥测
    App/app.cpp
    App/app_tasks.cpp
    App/command_router.cpp
    App/control_loop.cpp
    App/monitor.cpp
    App/protocol.cpp
    App/telemetry.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Framework
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform
    ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/App
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
