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

# ============================================================================
# 可复用组件（静态库）—— 2026-11-12 按学长建议加
#
# ⚠️ 这不是"提交一个 .a"。任务书 §2.4 禁止项 2 禁止提交二进制库，
#    这里的 .a 由 CMake 每次构建时从源码编出来，落在 build/ 里，
#    而 build/ 已在 .gitignore 第 2 行。仓库里永远只有 .c/.h 和这份文件。
#
# 原理（"切成库"到底做了什么）：
#   改之前：所有 .c 各自编成 .obj，再一次性链接成 rm_arm.elf
#   改之后：下面这些 .c 各自编成 .obj，先 arm-none-eabi-ar 打成 .a，
#           链接时把 .a 加进命令行末尾。**.c 文件本身一行都不用改。**
# ============================================================================

# ---- 库 1：协议编解码 ------------------------------------------------------
# proto_frame.c 只 include <string.h> 和 proto.h，不碰任何 HAL 类型，
# 因此可以单独拿到 PC 上用 gcc 编译 + 跑单元测试。这是"可复用"的硬证据。
add_library(proto_core STATIC
    Core/Src/proto_frame.c    # T5a: CRC8 / 组帧 / 流式解帧（纯逻辑，可 PC 上测）
)
target_include_directories(proto_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
)

# ---- 库 2：舵机驱动 --------------------------------------------------------
# 对外契约：使用方必须提供一个已初始化的全局 TIM_HandleTypeDef htim2
# （4 通道 50Hz PWM，由 main.c 定义）。本库只往 CCR 写值，不初始化定时器。
# 这是"由使用方提供依赖"的静态写法，不是耦合错误 —— 类比 FreeRTOS 要求
# 用户提供 vApplicationStackOverflowHook。
add_library(arm_servo STATIC
    Core/Src/arm_cfg.c        # T1: 四个关节的标定表（标定就是改这个文件）
    Core/Src/arm_pwm.c        # T1: 角度 → 脉宽 → CCR 的换算与输出
    Core/Src/arm_joint.c      # T2: 关节执行层（底座闭环 / 其余三路直通）
    Core/Src/as5600.c         # T2: 底座角度反馈（AS5600 磁编码器）
)
# 为什么手写下面 5 条 include，而不是 target_link_libraries(arm_servo stm32cubemx)：
# stm32cubemx 是 INTERFACE 库，除了 include 路径还挂了 target_sources()
# （main.c / HAL / 启动文件）。一旦 link 进来，那些 .c 会被**再编一遍**
# 塞进 libarm_servo.a，得到一个带 main.o 和 Reset_Handler 的畸形库。
# 而 cmake/stm32cubemx/CMakeLists.txt 是任务书禁止项 5（不许改），只能手写。
target_include_directories(arm_servo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
)
target_compile_definitions(arm_servo PUBLIC USE_HAL_DRIVER STM32F103xB)

# ---- ⚠️ 优化档位必须逐个 target 补 -----------------------------------------
# cmake/gcc-arm-none-eabi.cmake 第 31-33 行给 Debug 全局加了 -O0 -g3；
# 后面那个 -Og 是用 target_compile_options(${CMAKE_PROJECT_NAME}) 挂的，
# **只作用于 rm_arm 这一个 target**。新加的库不补，就会按 -O0 编 ——
# 那是当初 62088 B / 94.74%、只剩 3.4 KB 的那一档。
foreach(_lib proto_core arm_servo)
    target_compile_options(${_lib} PRIVATE
        $<$<CONFIG:Release>:-O2>
        $<$<CONFIG:Debug>:-Og>
    )
endforeach()

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # 下面这些是"应用层"，调库但不属于库：
    Core/Src/mpu6050.c        # T4: 姿态传感器（加速度/角速度/倾角 + 零偏标定）
    Core/Src/proto_link.c     # T5a: CDC 独占出入口（绑 USB 中间件，暂不拆库）
    Core/Src/proto_app.c      # T5a: 0x10 上报 + 0x20/21/22/23/24 处理
)
# 已拆成静态库的源文件见上方 add_library()：
#   proto_core ← proto_frame.c
#   arm_servo  ← arm_cfg.c / arm_pwm.c / arm_joint.c / as5600.c

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
    arm_servo       # 舵机驱动静态库（上方 add_library 定义）
    proto_core      # 协议编解码静态库（上方 add_library 定义）

    # Add user defined libraries
)
