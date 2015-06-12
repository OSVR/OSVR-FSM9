#
# FSM-9 plugin for OSVR
#
cmake_minimum_required(VERSION 2.8.12)
project(com_osvr_Freespace)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(osvr REQUIRED)
find_package(LibFreespace REQUIRED)

# Generate our JSON header file
osvr_convert_json(com_osvr_Freespace_json
    com_osvr_Freespace.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_Freespace_json.h")
	
# Be able to find our generated header file.
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# Build and install the plugin
osvr_add_plugin(com_osvr_Freespace 
	com_osvr_Freespace.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_Freespace_json.h")

target_include_directories(com_osvr_Freespace
    PRIVATE
	${LIBFREESPACE_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(com_osvr_Freespace
    osvr::osvrPluginKitCpp
	${LIBFREESPACE_LIBRARY})

install(FILES
    CONTRIBUTING.md
    README.md
    LICENSE
    NOTICE
    DESTINATION .)

