# The folder containing the bindings Python modules
set(BINDINGS_MODULES_DIR "${PROJECT_BINARY_DIR}/bindings")

# =============
# APIDOC TARGET
# =============

# This target generates the apidoc files of the Python modules and stores
# them in the docs/sphinx/ source folder.
# In order to make the multiversion website work, they need to be committed
# when they change.
#
# The files in the following folders are replaced:
#
# - docs/sphinx/scenario
# - docs/sphinx/gym_ignition
# - docs/sphinx/gym_ignition_environments
if (NOT TARGET ScenarioSwig::Core)
    message(FATAL_ERROR "Target ScenarioSwig::Core not found")
endif()

if (NOT TARGET ScenarioSwig::Monopod)
    message(FATAL_ERROR "Target ScenarioSwig::Monopod not found")
endif()


find_package(SphinxApidoc REQUIRED)
add_custom_target(apidoc ALL DEPENDS doxygen ScenarioSwig::Core ScenarioSwig::Monopod)
set(SPHINX_BUILD ${CMAKE_BINARY_DIR}/html)
set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_command(
    TARGET apidoc POST_BUILD
    COMMAND
    ${SPHINX_APIDOC_EXECUTABLE} --force
    --no-toc --module-first --maxdepth 4
    -t ${CMAKE_CURRENT_SOURCE_DIR}/_templates
    -o ${SPHINX_SOURCE}/scenario_monopod_docs/python
    ${BINDINGS_MODULES_DIR}/scenario/
    COMMENT "Building <scenario> apidoc")


# =============
# SPHINX TARGET
# =============

find_package(Sphinx REQUIRED)
find_package(SphinxMultiVersion REQUIRED)

if(${DOXYGEN_OUTPUT_DIRECTORY} STREQUAL "")
    message(FATAL_ERROR "Doxygen was not configured properly")
endif()

# =============
# Create built package for overlord webpage in /html
# =============

add_custom_target(source_rst_copy ALL
    DEPENDS apidoc
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${SPHINX_SOURCE}/scenario_monopod_docs" "${SPHINX_BUILD}/sphinx_build/scenario_monopod_docs"
    COMMENT "copy source doc build files."
  )

add_custom_target(index_copy ALL
    DEPENDS apidoc
    COMMAND ${CMAKE_COMMAND} -E copy "${SPHINX_SOURCE}/index.rst" "${SPHINX_BUILD}/sphinx_build/scenario_monopod_index.rst"
    COMMENT "copy index rst."
  )

add_custom_target(doxygen_xml_copy ALL
    DEPENDS apidoc
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${DOXYGEN_OUTPUT_DIRECTORY}/xml" "${SPHINX_BUILD}/sphinx_build/scenario_monopod_docs/xml"
    COMMENT "copy source doxygen xml build files."
  )


# Extend the Python path including before generating the website
# set(SPHINX_CMD export PYTHONPATH="${BINDINGS_MODULES_DIR}:$ENV{PYTHONPATH}" &&)
set(SPHINX_CMD export PYTHONPATH="${BINDINGS_MODULES_DIR}" &&)

# Sphinx build command
list(APPEND SPHINX_CMD
    ${SPHINX_MULTIVERSION_EXECUTABLE}
    ${SPHINX_SOURCE} ${SPHINX_BUILD}
    -D breathe_projects.scenario="${DOXYGEN_OUTPUT_DIRECTORY}/xml")

# Generate the website
add_custom_target(sphinx ALL
    DEPENDS apidoc
    COMMAND ${SPHINX_CMD}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating documentation with Sphinx")

# Disable GitHub pages autogeneration
add_custom_command(
    TARGET sphinx POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E touch "${SPHINX_BUILD}/.nojekyll"
    COMMENT "Disabling Jekyll in html folder")

# Handle redirect
add_custom_command(
    TARGET sphinx POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/index.html" "${SPHINX_BUILD}"
    COMMENT "Copying html redirect to html folder")
