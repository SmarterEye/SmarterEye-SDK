cmake_minimum_required(VERSION 3.1)

project(ImageStorageDemo)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories("../../inc")

message("Current folder is: ${CMAKE_CURRENT_LIST_DIR}")

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check https://doc.qt.io/qt/deployment-android.html for more information.
# They need to be set before the find_package( ...) calls below.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()


SET(CMAKE_INCLUDE_CURRENT_DIR ON)


if (WIN32)
  # set stuff for windows
    if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR "Only x64 is supported!\n please use '-G 'Visual Studio 15 2017 Win64'' as argument")
    endif()
    #set your opencv lib path here
    SET(OpenCV_DIR "D:/OpenCV 3.4.1/rebuild/install/x64/vc15/lib")
else()
  # set stuff for other systems
  # if you build custom opencv in linux, set following OpenCV_Dir
  # if your opencv install to system default path, like "/usr/local/lib" or others, maybe you need not set it
  # SET(OpenCV_DIR "/path/to/your/opencv")
  set(CMAKE_PREFIX_PATH /opt/Qt5.12.2/5.12.2/gcc_64) #$ENV{QT_DIR} "/usr/lib/x86_64_gnu_linux/cmake"

endif()

message(status " CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")

#find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
#find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
#find_package(Qt5 REQUIRED Widgets Core Network)
#find_package(Qt5Widgets REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core Network RemoteObjects REQUIRED)
# find opencv
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_DIR})

if (WIN32)
    link_libraries(opencv_core341 opencv_imgproc341 opencv_highgui341)
else()
    link_libraries(opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs pthread)
endif()

set(BIN_DIR ${CMAKE_CURRENT_LIST_DIR}/../../Runtime/Bin)

link_directories(${BIN_DIR})
link_libraries(StereoCamera ImageUtils)


set(PROJECT_SOURCES
        main.cpp
        frameselector.cpp
        frameselector.h
        imagestorage.cpp
        imagestorage.h
)

add_executable(ImageStorageDemo
    main.cpp frameselector.cpp imagestorage.cpp)

target_link_libraries(ImageStorageDemo Qt5::Widgets Qt5::Core Qt5::Network Qt5::RemoteObjects)

set_target_properties(ImageStorageDemo
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${BIN_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${BIN_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR}
)

SET(CMAKE_BUILD_TYPE DEBUG)
