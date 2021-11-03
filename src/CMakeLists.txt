# ==============
# Find scanrio Core dependencies
# ==============
# Fix the path here...
set(ENV{CMAKE_PREFIX_PATH} ~/.local/lib/python3.8/site-packages/scenario)
find_package(Scenario COMPONENTS Core REQUIRED)

set(SCENARIO_COMPONENTS ScenarioCore)
set(SCENARIO_PRIVATE_DEPENDENCIES "")

add_subdirectory(monopod)
# add_subdirectory(core)

# Dummy meta target for the top-level EXPORT
add_library(Scenario INTERFACE)

install(TARGETS Scenario EXPORT ScenarioExport)


# install(
#     TARGETS
#     Scenario
#     EXPORT ScenarioExport
#     LIBRARY DESTINATION ${SCENARIO_INSTALL_LIBDIR}
#     ARCHIVE DESTINATION ${SCENARIO_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${SCENARIO_INSTALL_BINDIR}
#     PUBLIC_HEADER DESTINATION
#   ${SCENARIO_INSTALL_LIBDIR}/cmake/Scenario)

# install_basic_package_files(Scenario
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY AnyNewerVersion
#     EXPORT ScenarioExport
#     DEPENDENCIES ${SCENARIO_COMPONENTS}
#     PRIVATE_DEPENDENCIES ${SCENARIO_PRIVATE_DEPENDENCIES}
#     NAMESPACE Scenario::
#     NO_CHECK_REQUIRED_COMPONENTS_MACRO
#     INSTALL_DESTINATION
#     ${SCENARIO_INSTALL_LIBDIR}/cmake/Scenario)
