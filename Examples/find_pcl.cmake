if (WIN32)
    # set stuff for windows
    # set your pcl lib path here
else()
  # set stuff for other systems
  # set your pcl lib path here
endif()

find_package(PCL)

if (PCL_INCLUDE_DIRS AND PCL_LIBRARIES)
    set(PCL_FOUND TRUE)

    message(STATUS "PCL library status:")
    message(STATUS "version: ${PCL_VERSION}")
    message(STATUS "libraries: ${PCL_LIBRARIES}")
    message(STATUS "include path: ${PCL_INCLUDE_DIRS}")
endif()
