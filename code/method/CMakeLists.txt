cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

set(method_HEADERS
        Optimisation.h
        Noise.h
        )

set(method_SOURCES
        Optimisation.cpp
        Noise.cpp bounding_sphere.h)

add_library(${PROJECT_NAME} STATIC ${method_SOURCES} ${method_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Symmetrization")

include(../cmake/FindGUROBI.cmake)
if (GUROBI_FOUND)
    message(STATUS "Gurobi include dir: " ${GUROBI_INCLUDE_DIRS})
    message(STATUS "Gurobi libraries: " ${GUROBI_LIBRARIES})

    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_GUROBI)

    target_include_directories(${PROJECT_NAME} PRIVATE ${GUROBI_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}   ${GUROBI_LIBRARIES})
endif()
find_package(CGAL REQUIRED)
target_link_libraries(${PROJECT_NAME} easy3d::core easy3d::fileio  easy3d::renderer CGAL::CGAL )
