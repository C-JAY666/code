cmake_minimum_required(VERSION 3.12)
project(rmcs_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-packed-bitfield-compat")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

file (GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.c
)

ament_auto_add_library (
  ${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/librmcs)

include_directories(SYSTEM "/usr/include/libusb-1.0")


include_directories(${PROJECT_SOURCE_DIR}/src/hardware/unitree_legs)

# 检测系统架构并选择对应的库
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    set(UNITREE_LIB_PATH "${PROJECT_SOURCE_DIR}/src/hardware/lib/libUnitreeMotorSDK_Arm64.so")
    message(STATUS "Detected ARM64 architecture, using libUnitreeMotorSDK_Arm64.so")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64")
    set(UNITREE_LIB_PATH "${PROJECT_SOURCE_DIR}/src/hardware/lib/libUnitreeMotorSDK_Linux64.so")
    message(STATUS "Detected x86_64 architecture, using libUnitreeMotorSDK_Linux64.so")
else()
    message(WARNING "Unknown architecture ${CMAKE_SYSTEM_PROCESSOR}, defaulting to Linux64")
    set(UNITREE_LIB_PATH "${PROJECT_SOURCE_DIR}/src/hardware/lib/libUnitreeMotorSDK_Linux64.so")
endif()

# 链接 Unitree 库
target_link_libraries(${PROJECT_NAME} 
    -lusb-1.0
    ${UNITREE_LIB_PATH})

# ============ Unitree 支持结束 ============

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

ament_auto_package()
