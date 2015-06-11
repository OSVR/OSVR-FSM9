#
# FSM-9 plugin for OSVR
#
cmake_minimum_required(VERSION 2.8.12)
project(com_osvr_FSM9)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(osvr REQUIRED)
find_package(LibFreespace REQUIRED)
find_package(jsoncpp REQUIRED)

# Generate our JSON header file
osvr_convert_json(com_osvr_FSM9_json
    com_osvr_FSM9.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_FSM9_json.h")
	
# Be able to find our generated header file.
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# Build and install the plugin
osvr_add_plugin(com_osvr_FSM9 
	com_osvr_FSM9.cpp
    FreespaceTracker.h
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_FSM9_json.h")

target_include_directories(com_osvr_FSM9
    PRIVATE
	${JSONCPP_INCLUDE_DIR}
	${LIBFREESPACE_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(com_osvr_FSM9
    osvr::osvrPluginKitCpp
	${JSONCPP_LIBRARY}
	${LIBFREESPACE_LIBRARY})

install(FILES
    CONTRIBUTING.md
    README.md
    LICENSE
    NOTICE
    DESTINATION .)

