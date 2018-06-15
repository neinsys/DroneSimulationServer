cmake_minimum_required(VERSION 2.8)
project (drone_server)

add_subdirectory(find_path)
add_subdirectory(image/PCL_converter)

include_directories("${PROJECT_SOURCE_DIR}/find_path")
include_directories("${PROJECT_SOURCE_DIR}/image/PCL_converter")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

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
add_custom_command(OUTPUT insertImage.html
        COMMAND ${CMAKE_COMMAND} -E
        copy ${PROJECT_SOURCE_DIR}/html/insertImage.html ${CMAKE_CURRENT_BINARY_DIR}/html/insertImage.html
        DEPENDS ${PROJECT_SOURCE_DIR}/html/insertImage.html
        )
add_custom_target(insertImage_copy ALL DEPENDS insertImage.html)


