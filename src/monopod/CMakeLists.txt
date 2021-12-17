
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

target_link_libraries(ScenarioMonopod
    PUBLIC
    ScenarioCore::ScenarioABC
    MonopodSdkCore::MonopodSdkCore
    PRIVATE
    ScenarioCore::CoreUtils
    # ${dependency path}
  )

# target_link_libraries( ScenarioMonopod
#     PRIVATE
#     real_time_tools::real_time_tools
#   )
set(misc_targets)
macro(add_demo demo_name)
  add_executable(
    ${demo_name}
    demos/${demo_name}.cpp)
  target_include_directories(
    ${demo_name}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/demos>
           $<INSTALL_INTERFACE:${SCENARIO_INSTALL_INCLUDEDIR}>)
  target_link_libraries(${demo_name} ScenarioMonopod)
  list(APPEND misc_targets ${demo_name})
endmacro()

add_demo(demo_main)

set_target_properties(ScenarioMonopod PROPERTIES
    PUBLIC_HEADER "${SCENARIO_MONOPOD_PUBLIC_HDRS}")


install(
    TARGETS
    ScenarioMonopod ${misc_targets}
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
