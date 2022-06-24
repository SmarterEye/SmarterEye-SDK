cmake_minimum_required(VERSION 3.0)

project(Disparity2PointCloud)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


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
    #set your opencv lib path here
    SET(OpenCV_DIR "D:/OpenCV 3.4.1/rebuild/install/x64/vc15/lib")
else()
  # set stuff for other systems
    add_definitions(-std=c++11)
    set(CMAKE_PREFIX_PATH /opt/Qt5.12.2/5.12.2/gcc_64)
endif()

find_package(Qt5 COMPONENTS Widgets Core Network RemoteObjects REQUIRED)
# find opencv
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_DIR})

if (WIN32)
    link_libraries(opencv_core341 opencv_imgproc341 opencv_highgui341 opencv_imgcodecs341)
else()
    link_libraries(opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_imgcodecs pthread)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../../Runtime/DBin")
else()
    set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../../Runtime/Bin")
endif()

link_directories("${BIN_DIR}")
link_libraries(StereoCamera ImageUtils)
############################################################
# Create an executable
############################################################

# Add an executable with the above sources
add_executable(Disparity2PointCloud
    main.cpp disparityframeconvertor.cpp disparityframeconvertor.h convertormanage.cpp convertormanage.h
    calibparamsgenerator.cpp calibparamsgenerator.h pcviewer.cpp pcviewer.h
)

target_link_libraries(Disparity2PointCloud Qt5::Widgets Qt5::Core Qt5::Network Qt5::RemoteObjects)

set_target_properties(Disparity2PointCloud
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
