cmake_minimum_required(VERSION 3.2)
project(SAP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++0x" )
# for profiling
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fno-omit-frame-pointer" )

include_directories(/home/grynca/DEV/gamedev/base/src/)
include_directories(/home/grynca/DEV/gamedev/maths/src/)

set(SOURCE_FILES main.cpp include/SAP/SAPManager.h include/SAP/SAP.inl include/SAP/SAP_internal.h include/SAP/SAP_internal.inl include/SAP.h)
add_executable(SAP ${SOURCE_FILES})
