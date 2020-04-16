find_package(OpenCV REQUIRED)                       # OpenCV库
find_package (Threads REQUIRED)                     # thread线程支持
add_executable(stereoMatch ${DIR_SRCS})             # 生成可执行文件
set(CMAKE_BUILD_TYPE Release)                                         # 默认 Debug模式
if (CMAKE_BUILD_TYPE STREQUAL Debug)
    add_definitions(-DDEBUG)                                        # 添加Debug选项
elseif(CMAKE_BUILD_TYPRE STREQUAL Release)
    add_definitions(-O3)                                            # Release模式优化选项
endif()
target_sources(stereoMatch 
                PUBLIC
                    ${CMAKE_CURRENT_LIST_DIR}/include/cvui.h
                    ${CMAKE_CURRENT_LIST_DIR}/include/serial.h
                    ${CMAKE_CURRENT_LIST_DIR}/include/receiver.hpp
                    ${CMAKE_CURRENT_LIST_DIR}/include/stereoMatch.hpp
                PRIVATE
                    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
                    ${CMAKE_CURRENT_LIST_DIR}/src/serial.cpp
                    ${CMAKE_CURRENT_LIST_DIR}/src/receiver.cpp
                    ${CMAKE_CURRENT_LIST_DIR}/src/stereoMatch.cpp
                )
target_include_directories(stereoMatch
                PUBLIC
                    ${CMAKE_CURRENT_LIST_DIR}/include
                )
target_link_libraries(stereoMatch ${OpenCV_LIBS})   # 链接OpenCV库