#
# FSM-9 plugin for OSVR
#
cmake_minimum_required(VERSION 3.2)
project(com_osvr_Freespace)
set(CMAKE_CXX_STANDARD 11)

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

#target_compile_features(com_osvr_Freespace PRIVATE
#    cxx_auto_type
#    cxx_defaulted_functions
#    cxx_deleted_functions
#    cxx_explicit_conversions
#    cxx_range_for
#    cxx_rvalue_references)

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

