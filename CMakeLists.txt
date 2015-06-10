#
# FSM-9 plugin for OSVR
#
cmake_minimum_required(VERSION 2.8.12)
project(com_osvr_FSM9)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(osvr REQUIRED)
find_package(LibFreespace REQUIRED)
find_package(VRPN REQUIRED)
find_package(jsoncpp REQUIRED)


osvr_convert_json(com_osvr_FSM9_json
    com_osvr_FSM9.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_FSM9_json.h")

osvr_add_plugin(com_osvr_FSM9 com_osvr_FSM9.cpp
    DevicesWithParameters.cpp
    DevicesWithParameters.h
    VRPNMultiserver.cpp
    VRPNMultiserver.h
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_FSM9_json.h")

target_include_directories(com_osvr_FSM9
    PRIVATE
	${JSONCPP_INCLUDE_DIR}
	${LIBFREESPACE_INCLUDE_DIR}
    ${VRPN_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(com_osvr_FSM9
    osvr::osvrPluginKitCpp
    osvr::osvrVRPNServer
	${JSONCPP_LIBRARY}
	${LIBFREESPACE_LIBRARY}
    ${VRPN_SERVER_LIBRARIES})

install(FILES
    CONTRIBUTING.md
    README.md
    LICENSE
    NOTICE
    DESTINATION .)

