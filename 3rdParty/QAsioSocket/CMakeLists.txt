cmake_minimum_required(VERSION 3.0)

project(QAsioSocket LANGUAGES C CXX)

add_definitions(-DQASIOSOCKET_LIBRARY)
add_definitions(-DASIO_STANDALONE)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Core REQUIRED)

include_directories(${Qt5Core_INCLUDE_DIRS})

include_directories(include)
include_directories(src)
include_directories(src/include_asio)

set(HEADER_LIST
    src/ioserviceclass.h
    src/linstenclass.hpp
    src/qsocketconnection.h
    include/tcpabstractsocket.h
    include/qasiotcpsocket.h
    include/qasiotcpserver.h
    include/qasiosocket.h)

set (SRC_LIST
        src/qasiotcpsocket.cpp
        src/qasiotcpserver.cpp
        src/qsocketconnection.cpp
        )

if(UNIX)
    qt5_wrap_cpp(MOC_SRC ${HEADER_LIST} ${SRC_LIST})
endif()

add_library(QAsioSocket SHARED ${HEADER_LIST} ${SRC_LIST} ${MOC_SRC})
target_link_libraries(QAsioSocket Qt5::Core)
