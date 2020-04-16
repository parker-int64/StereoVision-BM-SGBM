cmake_minimum_required(VERSION 3.0.0)                               # CMake最小版本号
project(StereoMatch VERSION 0.2.0)                                  # 项目名称
set(CPACK_PROJECT_NAME ${StereoMatch})                              
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})                       # 项目版本
set(CMAKE_CXX_COMPILER "g++")                                       # 设置编译器
set(EXECUTABLE_OUTPUT_PATH ../bin)                                  # 二进制文件安装目录
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread" )      # 编译指令                               

include(${CMAKE_CURRENT_LIST_DIR}/socketServer/CMakeLists.txt)      # UDP数据包接收
include(${CMAKE_CURRENT_LIST_DIR}/stereoMatch/CMakeLists.txt)        # 双目匹配+串口发送



