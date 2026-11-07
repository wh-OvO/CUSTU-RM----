cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
# 任务书 2.2「构建要求」明确：C 标准 C17（CMAKE_C_STANDARD 17）。
# CubeMX 默认写的是 11，这里按任务书改掉。C_EXTENSIONS ON 会得到 -std=gnu17。
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME rm_arm)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
# 语言必须在 project() 里一次声明：先 enable_language 再 project 会触发
# CMake 的 dev 警告 "project() should be called prior to enable_language()"。
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

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
    Core/Src/arm_cfg.c        # T1: 四个关节的标定表（标定就是改这个文件）
    Core/Src/arm_pwm.c        # T1: 角度 → 脉宽 → CCR 的换算与输出
    Core/Src/proto_frame.c    # T5a: CRC8 / 组帧 / 流式解帧（纯逻辑，可 PC 上测）
    Core/Src/proto_link.c     # T5a: CDC 独占出入口（环形缓冲 + 单帧发送）
    Core/Src/proto_app.c      # T5a: 0x10 上报 + 0x20/21/22/23/24 处理
)

# 任务书 2.2：「优化 Release 配置 -O2」。
# 注意不能去改 cmake/gcc-arm-none-eabi.cmake —— 任务书明令该文件
# 「由 CubeMX 生成，不得手改」，而且每次 GENERATE CODE 都会被覆盖。
# CubeMX 默认给 Release 的是 -Os，这里用 target 级选项补一个 -O2：
# gcc 命令行上后出现的 -O 会覆盖前面的 -O。
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:-O2>
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
