cmake_minimum_required(VERSION 3.0)

project(GetPerceptionDemo)

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_CURRENT_LIST_DIR}/../../inc)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../../3rdParty/QAsioSocket/src/include_asio)

message("Current folder is: ${CMAKE_CURRENT_LIST_DIR}")

if (WIN32)
  # set stuff for windows
    add_compile_definitions(ASIO_STANDALONE = 1)
    if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR "Only x64 is supported!\n please use '-G 'Visual Studio 15 2017 Win64'' as argument")
    endif()
    add_definitions(-D_WIN32_WINNT=0x0601)
else()
  # set stuff for other systems
    add_definitions(-std=c++11)
endif()

#if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#    set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../../Runtime/DBin")
#else()
    set(BIN_DIR ${CMAKE_CURRENT_LIST_DIR}/../../Runtime/Bin)
#endif()

link_directories(${BIN_DIR})

link_libraries(QAsioSocket)
link_libraries(SeUdpClient)

############################################################
# Create an executable
############################################################

# Add an executable with the above sources
add_executable(GetPerceptionDemo
    main.cpp dataviewer.cpp
    dataviewer.h
    )


set_target_properties(GetPerceptionDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
    COMPILE_FLAGS "-pthread"
    LINK_FLAGS "-pthread"
)
