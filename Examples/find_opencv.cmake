if (WIN32)
    # set stuff for windows
    # set your opencv lib path here
    # SET(OpenCV_DIR "C:/opencv_341/build/install/x64/vc15/lib")
else()
  # set stuff for other systems
  # if you build custom opencv in linux, set following OpenCV_Dir
  # if your opencv install to system default path, like "/usr/local/lib" or others, maybe you need not set it
  # SET(OpenCV_DIR "/path/to/your/opencv")
endif()

find_package(OpenCV)

if (OpenCV_INCLUDE_DIRS AND OpenCV_LIBS)
    set(OPENCV_FOUND TRUE)
endif()

message(STATUS "OpenCV library status:")
message(STATUS "version: ${OpenCV_VERSION}")
message(STATUS "libraries: ${OpenCV_LIBS}")
message(STATUS "include path: ${OpenCV_INCLUDE_DIRS}")
