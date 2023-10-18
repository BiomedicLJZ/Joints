cmake_minimum_required(VERSION 3.26)
project(Joints)

set(CMAKE_CXX_STANDARD 17)

add_executable(Joints Revolute.cpp)
find_package(Box2d CONFIG REQUIRED)
target_link_libraries(Joints PRIVATE box2d::box2d)
find_package(SFML COMPONENTS system window graphics CONFIG REQUIRED)
target_link_libraries(Joints PRIVATE sfml-system sfml-network sfml-graphics sfml-window)
