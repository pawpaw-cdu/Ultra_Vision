cmake_minimum_required(VERSION 3.10)
project(buff_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 大恒 SDK 路径（根据你的实际位置修改）
# set(DAHENG_SDK_ROOT "/home/pawpaw-ubuntu/rm_ultra/io/GalaxySDK")
# set(DAHENG_INC_DIR "${DAHENG_SDK_ROOT}/include")
# set(DAHENG_LIB_DIR "${DAHENG_SDK_ROOT}/lib/armv8")

# 源文件列表
set(SOURCES
    ../energy_node.cpp
)

find_package(OpenCV REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# 查找大恒库（直接指定绝对路径，避免 find_library 出错）
# set(GxIAPI_LIB "${DAHENG_LIB_DIR}/libgxiapi.so")
# if(NOT EXISTS ${GxIAPI_LIB})
#     message(FATAL_ERROR "Cannot find libgxiapi.so at ${GxIAPI_LIB}")
# endif()
# message(STATUS "Found GxIAPI: ${GxIAPI_LIB}")

# 创建可执行文件
# add_executable(buff ${SOURCES})

add_executable(energy_demo ../energy_node.cpp )
target_link_libraries(energy_demo ${OpenCV_LIBS})

# 包含头文件目录
target_include_directories(energy_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    # ${DAHENG_INC_DIR}
    ${OpenCV_INCLUDE_DIRS}
    # ${YAML_CPP_INCLUDE_DIRS}
)

# 链接库（注意：GxIAPI_LIB 必须出现在这里）
target_link_libraries(energy_demo
    ${OpenCV_LIBS}
    # ${YAML_CPP_LIBRARIES}
    # ${GxIAPI_LIB}
)