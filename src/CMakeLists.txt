cmake_minimum_required(VERSION 2.8)
project (drone_server)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(drone_server main.cpp)
target_link_libraries(drone_server ${Boost_LIBRARIES})
target_link_libraries(drone_server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(drone_server ${PCL_LIBRARIES})
