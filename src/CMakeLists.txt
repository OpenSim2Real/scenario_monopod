# ==============
# Find scanrio Monopod dependencies
# ==============

set(SCENARIO_COMPONENTS ScenarioCore)
set(SCENARIO_PRIVATE_DEPENDENCIES "")

add_subdirectory(monopod)
list(APPEND SCENARIO_COMPONENTS ScenarioMonopod ScenarioGazebo)

# Dummy meta target for the top-level EXPORT
add_library(Scenario INTERFACE)

install(TARGETS Scenario EXPORT ScenarioExport)


install_basic_package_files(Scenario
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT ScenarioExport
    DEPENDENCIES ${SCENARIO_COMPONENTS}
    PRIVATE_DEPENDENCIES ${SCENARIO_PRIVATE_DEPENDENCIES}
    NAMESPACE Scenario::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${SCENARIO_INSTALL_LIBDIR}/cmake/Scenario)
