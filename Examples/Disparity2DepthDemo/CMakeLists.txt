cmake_minimum_required(VERSION 3.1)

project(Disparity2DepthDemo)

set(CMAKE_CXX_STANDARD 11)

include_directories("../../inc")

message("Current folder is: ${CMAKE_CURRENT_LIST_DIR}")

if (WIN32)
  # set stuff for windows
    if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR "Only x64 is supported!\n please use '-G 'Visual Studio 15 2017 Win64'' as argument")
    endif()
    #set your opencv lib path here
    SET(OpenCV_DIR "C:/opencv_341/build/install/x64/vc15/lib")
else()
  # set stuff for other systems
  # if you build custom opencv in linux, set following OpenCV_Dir
  # if your opencv install to system default path, like "/usr/local/lib" or others, maybe you need not set it
  # SET(OpenCV_DIR "/path/to/your/opencv")
endif()

# find opencv
find_package(OpenCV REQUIRED)
#message(STATUS "OpenCV library status:")
#message(STATUS "version: ${OpenCV_VERSION}")
#message(STATUS "libraries: ${OpenCV_LIBS}")
#message(STATUS "include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_DIR})

if (WIN32)
    link_libraries(opencv_core341 opencv_imgproc341 opencv_highgui341)
else()
    link_libraries(opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs pthread)
endif()


#if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#    set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../../Runtime/DBin")
#else()
    set(BIN_DIR ${CMAKE_CURRENT_LIST_DIR}/../../Runtime/Bin)
#endif()

link_directories(${BIN_DIR})
link_libraries(StereoCamera ImageUtils)
############################################################
# Create an executable
############################################################

# Add an executable with the above sources
add_executable(Disparity2DepthDemo
    main.cpp framemonitor.cpp
)

set_target_properties(Disparity2DepthDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${BIN_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${BIN_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR}
)
