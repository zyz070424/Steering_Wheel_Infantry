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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Gimbal)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

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
    # Add user sources here
    user_file/1_BSP/Driver/FDCAN/drv_fdcan.cpp
    user_file/1_BSP/Driver/UART/drv_uart.cpp
    user_file/1_BSP/Driver/SPI/drv_spi.cpp
    user_file/1_BSP/Driver/PWM/drv_pwm.cpp
    user_file/1_BSP/Driver/DWT/drv_dwt.cpp
    user_file/1_BSP/Driver/USB/drv_usb.cpp
    user_file/1_BSP/Algorthm/KalmanFilter/alg_arm_math_compat.c
    user_file/1_BSP/Algorthm/KalmanFilter/alg_kalman_filter.c
    user_file/1_BSP/Algorthm/QuaternionEKF/alg_quaternion_ekf.c
    user_file/1_BSP/Algorthm/PID/alg_pid.cpp
    user_file/1_BSP/Algorthm/CommonMath/common_math.cpp

    user_file/2_Device/MOTOR/DMMOTOR/dvc_dmmotor.cpp
    user_file/2_Device/MOTOR/DJIMOTOR/dvc_djimotor.cpp
    user_file/2_Device/BUZZER/dvc_buzzer.cpp
    user_file/2_Device/BMI088/dvc_bmi088.cpp
    user_file/2_Device/DR16/dvc_dr16.cpp
    user_file/2_Device/FDCAN_COMM/dvc_fdcan_comm.cpp
    user_file/2_Device/VISION/dvc_vision.cpp

    user_file/3_Application/IMU/app_imu.cpp
    user_file/3_Application/Config/app_config.cpp
    user_file/3_Application/TASK/mytask.cpp
    user_file/3_Application/CMD/app_cmd.cpp
    user_file/3_Application/GIMBAL/app_gimbal.cpp
    user_file/3_Application/FEED_ROTOR/app_feed_rotor.cpp
    user_file/3_Application/SHOOT/app_shoot.cpp
    

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    user_file/1_BSP/Driver/FDCAN
    user_file/1_BSP/Driver/UART
    user_file/1_BSP/Driver/SPI
    user_file/1_BSP/Driver/PWM
    user_file/1_BSP/Driver/DWT
    user_file/1_BSP/Driver/USB
    user_file/1_BSP/Algorthm/KalmanFilter
    user_file/1_BSP/Algorthm/QuaternionEKF
    user_file/1_BSP/Algorthm/PID
    user_file/1_BSP/Algorthm/Matrix
    user_file/1_BSP/Algorthm/CommonMath

    user_file/2_Device/MOTOR
    user_file/2_Device/MOTOR/DMMOTOR
    user_file/2_Device/MOTOR/DJIMOTOR
    user_file/2_Device/BUZZER
    user_file/2_Device/BMI088
    user_file/2_Device/DR16
    user_file/2_Device/FDCAN_COMM
    user_file/2_Device/VISION

    user_file/3_Application/IMU
    user_file/3_Application/CONFIG
    user_file/3_Application/TASK
    user_file/3_Application/GIMBAL
    user_file/3_Application/FEED_ROTOR
    user_file/3_Application/SHOOT
    user_file/3_Application/CMD
    ../../Common

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
