cmake_minimum_required(VERSION 3.2)
project(test_Phy)

add_executable(test_Phy main.cpp)

target_link_libraries(test_Phy Phy)