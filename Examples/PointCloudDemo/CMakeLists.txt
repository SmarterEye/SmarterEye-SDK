cmake_minimum_required(VERSION 3.0)

project(PointCloudDemo)

find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_libraries(${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

include_directories("../../inc")

message("Current folder is: ${CMAKE_CURRENT_LIST_DIR}")

if (WIN32)
  # set stuff for windows
    if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR "Only x64 is supported!\n please use '-G 'Visual Studio 15 2017 Win64'' as argument")
    endif()
else()
  # set stuff for other systems
    add_definitions(-std=c++11)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../../Runtime/DBin")
else()
    set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../../Runtime/Bin")
endif()

link_directories("${BIN_DIR}")
link_libraries(StereoCamera ImageUtils pthread)
############################################################
# Create an executable
############################################################

# Add an executable with the above sources
add_executable(PointCloudDemo
    main.cpp mycamerahandler.cpp pcviewer.cpp
    mycamerahandler.h pcviewer.h
)

set_target_properties(PointCloudDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
