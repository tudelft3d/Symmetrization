cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

add_executable(${PROJECT_NAME}
        Symmetry_viewer.h
        Symmetry_viewer.cpp
        main.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Symmetrization")
target_link_libraries(${PROJECT_NAME}  easy3d_fileio   easy3d::viewer easy3d::core easy3d::renderer  easy3d::util method 3rd_imgui)
