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
    Core/Src/arm_joint.c      # T2: 关节执行层（底座闭环 / 其余三路直通）
    Core/Src/as5600.c         # T2: 底座角度反馈（AS5600 磁编码器）
    Core/Src/mpu6050.c        # T4: 姿态传感器（加速度/角速度/倾角 + 零偏标定）
    Core/Src/proto_frame.c    # T5a: CRC8 / 组帧 / 流式解帧（纯逻辑，可 PC 上测）
    Core/Src/proto_link.c     # T5a: CDC 独占出入口（环形缓冲 + 单帧发送）
    Core/Src/proto_app.c      # T5a: 0x10 上报 + 0x20/21/22/23/24 处理
)

# 任务书 2.2：「优化 Release 配置 -O2」。
# 注意不能去改 cmake/gcc-arm-none-eabi.cmake —— 任务书明令该文件
# 「由 CubeMX 生成，不得手改」，而且每次 GENERATE CODE 都会被覆盖。
# CubeMX 默认给 Release 的是 -Os，这里用 target 级选项补一个 -O2：
# gcc 命令行上后出现的 -O 会覆盖前面的 -O。
#
# 同理，Debug 的 -O0 也在这里覆盖成 -Og。原因（2026-11-10 实测）：
#   -O0 -g3  → 62088 B / 94.74%（只剩 3.4 KB，加一点点功能就链接失败）
#   -Og -g3  → 39584 B / 60.40%（剩 25.3 KB）
#   -O2      → 37168 B / 56.71%
# -O0 的全量代码是 60 KB，不是"代码本来就大" —— 是 -O0 不做任何优化撑出来的。
# 而 -Og 是 gcc 专为调试加的档位：变量不丢、不重排、单步不跳，调试体验和 -O0
# 基本一样，只是省掉 22 KB。**别用 -O2 调程序**，变量会被优化没，单步会乱跳。
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:-O2>
    $<$<CONFIG:Debug>:-Og>
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
