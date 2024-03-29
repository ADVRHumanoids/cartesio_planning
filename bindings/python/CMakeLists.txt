cmake_minimum_required(VERSION 3.15)

find_package(Python3 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11 REQUIRED)
message(STATUS "**********************************Check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "+++++++++++++++++++++++++++++++Compiling python bindings (${pybind11_LIBRARIES})")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    pybind11_add_module(planning cartesio_planning.cpp)
    target_link_libraries(planning PRIVATE cartesio_planning)
    install(TARGETS planning
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

    pybind11_add_module(validity_check validity_check.cpp)
    target_link_libraries(validity_check PRIVATE cartesio_planning)
    install(TARGETS validity_check
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

    pybind11_add_module(visual_tools visual_tools.cpp)
    target_link_libraries(visual_tools PRIVATE cartesio_planning)
    install(TARGETS visual_tools
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

    pybind11_add_module(constraints constraints.cpp)
    target_link_libraries(constraints PRIVATE cartesio_planning)
    install(TARGETS constraints
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
        
    pybind11_add_module(NSPG NSPG.cpp)
    target_link_libraries(NSPG PRIVATE cartesio_planning)
    install(TARGETS NSPG
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

else()
    message(STATUS "-----------------------------------pybind not found")
endif()

