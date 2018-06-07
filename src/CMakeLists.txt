cmake_minimum_required(VERSION 2.8)
project (drone_server)

add_subdirectory(find_path)
#add_subdirectory(image/PCL_converter)
include_directories("${PROJECT_SOURCE_DIR}/find_path")

add_executable(drone_server main.cpp)
target_link_libraries(drone_server ${Boost_LIBRARIES})
target_link_libraries(drone_server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(drone_server ${PCL_LIBRARIES})
target_link_libraries(drone_server find_path)
