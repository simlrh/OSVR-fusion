cmake_minimum_required(VERSION 2.8.12)
project(FusionPlugin) 

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH} )

find_package(osvr REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories("${EIGEN3_INCLUDE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")

osvr_convert_json(je_nourish_fusion_json
    je_nourish_fusion.json
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_fusion_json.h")

osvr_add_plugin(NAME je_nourish_fusion
    CPP
    SOURCES
    je_nourish_fusion.cpp
	PositionReader.h
	PositionReader.cpp
	OrientationReader.h
	OrientationReader.cpp
	stdafx.h
	FusionMath.h
	FusionMath.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_fusion_json.h")

target_link_libraries(je_nourish_fusion osvr::osvrClientKitCpp osvr::osvrAnalysisPluginKit jsoncpp_lib)
