set(CMAKE_INCLUDE_CURRENT_DIR ON)

# debug
if(NOT CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX d)
endif()

if(DEBUG)
    add_definitions(-DDEBUG)
    message(STATUS "Using macro DEBUG")
endif()

set(__expr ".so*")
if(OS_WIN)
    set(__expr "*.dll")
elseif(OS_MAC)
    set(__expr "*.dylib")
endif()

# flags

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
