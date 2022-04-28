if (WIN32)
    # set stuff for windows
#    SET(Qt5_PATH /path/to/your/qt/lib)
else()
    # set stuff for other systems
     SET(Qt5_PATH /opt/Qt5.12.2/5.12.2/gcc_64)
endif()

SET(CMAKE_PREFIX_PATH ${Qt5_PATH})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Core Network RemoteObjects REQUIRED)

include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Network_INCLUDE_DIRS})
