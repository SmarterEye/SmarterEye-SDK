cmake_minimum_required(VERSION 3.0)

project(inc LANGUAGES C CXX)
set(SDK_VERSION 0.2.25)
add_definitions(-std=c++11)

include(cmake/common.cmake)

# config

set(SDK_NAME "${PROJECT_NAME}")
set(SDK_ROOT "${PROJECT_SOURCE_DIR}")

# packages

LIST(APPEND CMAKE_MODULE_PATH ${SDK_ROOT}/cmake)

include(cmake/find_qt.cmake)

include_directories(${SDK_ROOT}/inc)

include_directories(${SDK_ROOT}/3rdParty/QAsioSocket/include)

# src

add_definitions(-DSMARTER_ROS)
set(SDK_MODULES
    Satp
    SatpExt
    StereoCamera
    ImageUtils
    MessageBus
    RtdbService
    RtdbManager
    RealtimeDatabase
    )

add_definitions(
    -DSATP_LIBRARY
    -DSATPEXT_LIBRARY
    -DSTEREOCAMERA_LIBRARY
    -DIMAGEUTILS_LIBRARY
    -DMESSAGEBUS_LIBRARY
    -DMESSAGEBUS_RTDBSERVICE
    -DMESSAGEBUS_RTDBMANAGER
    -DMESSAGEBUS_REALTIMEDATABASE
    )


set(SDK_INCS)
foreach(MODULE_DIR ${SDK_MODULES})
    include_directories(${MODULE_DIR})
    aux_source_directory(${MODULE_DIR} SDK_SRCS)
    file(GLOB CUR_INCS "${MODULE_DIR}/*.h")
    list(APPEND SDK_INCS ${CUR_INCS})
endforeach()

# output

set(OUTPUT_DIR "${SDK_ROOT}/_output")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}/bin)

# submodule

add_subdirectory(${SDK_ROOT}/3rdParty/QAsioSocket)

set_target_properties(QAsioSocket
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}/bin
    )

# rpath
set(SDK_CMAKE_RPATH
    "${CMAKE_INSTALL_PREFIX}/lib"
    "${Qt5_PATH}/lib"
    )

# build shared

# QTRO
if(UNIX)
    qt5_wrap_cpp(MOC_SRC ${SDK_SRCS} ${SDK_INCS})
endif()

add_library(${SDK_NAME} SHARED ${SDK_SRCS} ${SDK_INCS} ${MOC_SRC})
add_dependencies(${SDK_NAME} QAsioSocket)
target_link_libraries(${SDK_NAME}
    PRIVATE $<BUILD_INTERFACE:QAsioSocket>
    Qt5::Core Qt5::Network Qt5::RemoteObjects
    )
set_target_properties(${SDK_NAME} PROPERTIES
    VERSION ${SDK_VERSION}
    INSTALL_RPATH "${SDK_CMAKE_RPATH}"
    )


# install

if(WIN32)
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${SDK_ROOT}/install" CACHE PATH "default install path" FORCE)
    endif()
endif()

set(SDK_CMAKE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/inc/${SDK_NAME}")
set(SDK_CMAKE_BINDIR "${CMAKE_INSTALL_PREFIX}/bin")
set(SDK_CMAKE_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")
set(SDK_CMAKE_INSTALLDIR "${SDK_CMAKE_LIBDIR}/cmake/${SDK_NAME}")

install(TARGETS ${SDK_NAME}
    EXPORT ${SDK_NAME}-targets
    ARCHIVE DESTINATION ${SDK_CMAKE_LIBDIR}
    LIBRARY DESTINATION ${SDK_CMAKE_LIBDIR}
    RUNTIME DESTINATION ${SDK_CMAKE_BINDIR}
    )

install(DIRECTORY
    "${SDK_ROOT}/inc"
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${SDK_NAME}-version.cmake
    VERSION ${SDK_VERSION}
    COMPATIBILITY SameMajorVersion
    )


install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SDK_NAME}-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${SDK_NAME}-version.cmake
  DESTINATION ${SDK_CMAKE_INSTALLDIR}
)
