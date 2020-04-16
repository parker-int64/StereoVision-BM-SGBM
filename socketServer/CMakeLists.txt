add_executable(socketServer)
find_package(OpenCV REQUIRED)
target_sources(socketServer
                PUBLIC
                    ${CMAKE_CURRENT_LIST_DIR}/include/socketServer.h
                PRIVATE 
                    ${CMAKE_CURRENT_LIST_DIR}/src/socketServer.cpp)
# include_directories()
target_include_directories(socketServer 
                PUBLIC
                    ${OpenCV_INCLUDE_DIRS} 
                    ${CMAKE_CURRENT_LIST_DIR}/include)
# target_link_libraries(socketServer wsock32)
target_link_libraries(socketServer 
                    wsock32
                    ${OpenCV_LIBS})