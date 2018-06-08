cmake_minimum_required(VERSION 2.8)
project (drone_server)

add_subdirectory(find_path)
add_subdirectory(image/PCL_converter)
include_directories("${PROJECT_SOURCE_DIR}/find_path")
include_directories("${PROJECT_SOURCE_DIR}/image/PCL_converter")


find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(drone_server main.cpp)
target_link_libraries(drone_server ${Boost_LIBRARIES})
target_link_libraries(drone_server ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(drone_server ${PCL_LIBRARIES})
target_link_libraries(drone_server find_path)
target_link_libraries(drone_server image_processing)
