# ==============
# Find scanrio Core dependencies
# ==============
# Fix the path here...
set(ENV{CMAKE_PREFIX_PATH} ~/.local/lib/python3.8/site-packages/scenario)
find_package(Scenario COMPONENTS Core REQUIRED)

# add_subdirectory(core)

set(SCENARIO_COMPONENTS ScenarioCore)
set(SCENARIO_PRIVATE_DEPENDENCIES "")

add_subdirectory(monopod)

# Dummy meta target for the top-level EXPORT
add_library(Scenario INTERFACE)

install(
    TARGETS Scenario
    EXPORT ScenarioExport)

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
