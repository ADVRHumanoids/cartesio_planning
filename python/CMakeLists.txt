cmake_policy(SET CMP0094 NEW)
find_package(Python3 COMPONENTS Interpreter Development)

find_package(pybind11 QUIET)

option(COMPILE_PYTHON_BINDINGS "Compile python bindings (requires pybind11)" TRUE)

if(${pybind11_FOUND} AND ${COMPILE_PYTHON_BINDINGS})

    message(STATUS "Found python (${Python3_EXECUTABLE}) will compile python bindings")

    # get python install path
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import site, os; print(os.path.relpath(site.USER_SITE, site.USER_BASE))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    message(STATUS "Python install dir: ${PYTHON_SITE}")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    pybind11_add_module(pycartesio_planning pycartesio_planning.cpp)

    target_link_libraries(pycartesio_planning PUBLIC cartesio_planning)

    install(TARGETS pycartesio_planning
        DESTINATION ${PYTHON_SITE}/cartesio_planning)

    install(FILES __init__.py DESTINATION ${PYTHON_SITE}/cartesio_planning)

endif()
