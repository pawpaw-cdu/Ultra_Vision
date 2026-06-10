cmake_minimum_required(VERSION 3.10)
project(auto_buff_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    ~/rm_ultra/src/auto_buff/src/detector_buff.cpp
    ~/rm_ultra/src/auto_buff/test/video_test.cpp
    ~/rm_ultra/src/auto_buff/src/base_buff.cpp
    ~/rm_ultra/src/auto_buff/src/config.cpp
    ~/rm_ultra/src/auto_buff/src/fanblade.cpp
    ~/rm_ultra/src/auto_buff/src/utils.cpp
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_package(Eigen3 REQUIRED)

add_executable(buff_test ${SOURCES})

target_include_directories(buff_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(buff_test
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)