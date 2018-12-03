cmake_minimum_required(VERSION 3.10)
project(LimbController_cpp)

set(CMAKE_CXX_STANDARD 11)

include_directories(../3rd_party_software/eigen)

add_executable(LimbController_cpp
        AlphredLimb.h
        AlphredLimb.cpp
        main.cpp)