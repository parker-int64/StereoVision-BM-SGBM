cmake_minimum_required(VERSION 3.0.0)
project(StereoMatch VERSION 0.2.0)
set(CMAKE_CXX_COMPILER "g++")
set(CPACK_PROJECT_NAME ${StereoMatch})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(EXECUTABLE_OUTPUT_PATH ../bin)                  # 二进制文件安装目录
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread" )                                      
set(CMAKE_BUILD_TYPE Debug)                         # 默认 Debug模式
if (CMAKE_BUILD_TYPE STREQUAL Debug)
    add_definitions(-DDEBUG)
elseif(CMAKE_BUILD_TYPRE STREQUAL Release)
    add_definitions(-O3)                            # Release模式优化选项
endif()
find_package(OpenCV REQUIRED)                       # OpenCV库
find_package (Threads REQUIRED)                     # thread线程支持
aux_source_directory(./src/ DIR_SRCS)               # 源文件目录
add_executable(stereoMatch ${DIR_SRCS})             # 生成可执行文件
include_directories(./include/)                     # 工作目录头文件
include_directories(${OpenCV_INCLUDE_DIRS})         # OpenCV的文件
target_link_libraries(stereoMatch ${OpenCV_LIBS})   # 链接OpenCV库