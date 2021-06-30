cmake_minimum_required(VERSION 3.5)
project(app)

find_package(ur_rtde REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE ur_rtde::rtde)
