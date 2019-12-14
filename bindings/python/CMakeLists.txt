cmake_minimum_required(VERSION 3.5)

find_package(pybind11 QUIET)
message(STATUS "**********************************Check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "+++++++++++++++++++++++++++++++Compiling python bindings")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyplan pyPlannerClient.cpp)
    target_link_libraries(pyplan PUBLIC cartesio_planning)

    install(TARGETS pyplan
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

    file(WRITE ${CMAKE_BINARY_DIR}/__init__.py "")
    install(FILES ${CMAKE_BINARY_DIR}/__init__.py DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})


else()
    message(STATUS "-----------------------------------pybind not found")
endif()
