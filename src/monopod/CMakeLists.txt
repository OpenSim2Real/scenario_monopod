
# ==============
# ScenarioMonopod
# ==============
#
set(SCENARIO_MONOPOD_PUBLIC_HDRS
    include/scenario/monopod/World.h
    include/scenario/monopod/Model.h
    include/scenario/monopod/Joint.h
    include/scenario/monopod/easylogging++.h)

add_library(ScenarioMonopod
    ${SCENARIO_MONOPOD_PUBLIC_HDRS}
    src/World.cpp
    src/Model.cpp
    src/Joint.cpp
    src/easylogging++.cc)

add_library(ScenarioMonopod::ScenarioMonopod ALIAS ScenarioMonopod)

target_include_directories(ScenarioMonopod PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${SCENARIO_INSTALL_INCLUDEDIR}>)

# find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
# execute_process(
#     COMMAND "${Python3_EXECUTABLE}" -c "import scenario; print(scenario.__path__[0])"
#     OUTPUT_VARIABLE Scenario_OUT_PATH
# )

# set(CMAKE_PREFIX_PATH "/home/capstone/.local/lib/python3.8/site-packages/scenario")
find_package(Scenario COMPONENTS Core REQUIRED NO_MODULE)

target_link_libraries(ScenarioMonopod
    PUBLIC
    ScenarioCore::ScenarioABC
    PRIVATE
    ScenarioCore::CoreUtils
    # ${dependency path}
  )

set_target_properties(ScenarioMonopod PROPERTIES
PUBLIC_HEADER "${SCENARIO_MONOPOD_PUBLIC_HDRS}")


install(
    TARGETS
    ScenarioMonopod
    EXPORT ScenarioMonopodExport
    LIBRARY DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${SCENARIO_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${SCENARIO_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION
    ${SCENARIO_INSTALL_INCLUDEDIR}/scenario/monopod)

# ===================
# Install the targets
# ===================

install_basic_package_files(ScenarioMonopod
    COMPONENT Monopod
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT ScenarioMonopodExport
    DEPENDENCIES ScenarioCore
    NAMESPACE ScenarioMonopod::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${SCENARIO_INSTALL_LIBDIR}/cmake/ScenarioMonopod
)
