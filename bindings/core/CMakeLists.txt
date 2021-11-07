# set(ENV{CMAKE_PREFIX_PATH} "~/.local/lib/python3.8/site-packages/scenario")
# # set(ENV{CMAKE_PREFIX_PATH} ${Python3_SITELIB}/scenario)
find_package(Scenario COMPONENTS Core REQUIRED)

set(scenario_swig_name "core")
set_source_files_properties(${scenario_swig_name}.i PROPERTIES CPLUSPLUS ON)

# The bindings shared library is stored in the Python package of the build tree
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../scenario/bindings)


swig_add_library(${scenario_swig_name}
    TYPE MODULE
    LANGUAGE python
    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/../scenario/bindings
    OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/..
    SOURCES ${scenario_swig_name}.i)

add_library(ScenarioSwig::Core ALIAS core)

# swig_link_libraries(${scenario_swig_name} PUBLIC ScenarioCore)
# target_link_libraries(${scenario_swig_name} PUBLIC ScenarioCore)

target_link_libraries(${scenario_swig_name} PUBLIC
    ScenarioCore::ScenarioABC
    ScenarioCore::CoreUtils
    Python3::Python)

set_property(TARGET ${scenario_swig_name} PROPERTY
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)

# https://github.com/swig/swig/issues/672#issuecomment-400577864
set_property(TARGET ${scenario_swig_name} PROPERTY
    SWIG_COMPILE_OPTIONS -doxygen -Dfinal)
