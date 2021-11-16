# Install script for directory: /home/capstone/Documents/Repos/SIM/src/monopod

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libScenarioMonopod.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libScenarioMonopod.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libScenarioMonopod.so"
         RPATH "\$ORIGIN/:\$ORIGIN/../../:\$ORIGIN/../../../local/lib:\$ORIGIN/../../../../../local/lib:\$ORIGIN/../lib:\$ORIGIN/scenario/plugins:\$ORIGIN/../../../local/lib/scenario/plugins:\$ORIGIN/../../../../../local/lib/scenario/plugins:\$ORIGIN/../lib/scenario/plugins:\$ORIGIN/../../lib/python3/dist-packages:\$ORIGIN/../../../../lib/python3/dist-packages:\$ORIGIN/../../lib/python3/dist-packages/scenario/bindings:\$ORIGIN/../../../../lib/python3/dist-packages/scenario/bindings:\$ORIGIN/scenario/bindings")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/capstone/Documents/Repos/SIM/lib/libScenarioMonopod.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libScenarioMonopod.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libScenarioMonopod.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libScenarioMonopod.so"
         OLD_RPATH "/home/capstone/Documents/Repos/SIM/lib:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "\$ORIGIN/:\$ORIGIN/../../:\$ORIGIN/../../../local/lib:\$ORIGIN/../../../../../local/lib:\$ORIGIN/../lib:\$ORIGIN/scenario/plugins:\$ORIGIN/../../../local/lib/scenario/plugins:\$ORIGIN/../../../../../local/lib/scenario/plugins:\$ORIGIN/../lib/scenario/plugins:\$ORIGIN/../../lib/python3/dist-packages:\$ORIGIN/../../../../lib/python3/dist-packages:\$ORIGIN/../../lib/python3/dist-packages/scenario/bindings:\$ORIGIN/../../../../lib/python3/dist-packages/scenario/bindings:\$ORIGIN/scenario/bindings")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libScenarioMonopod.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/scenario/monopod" TYPE FILE FILES
    "/home/capstone/Documents/Repos/SIM/src/monopod/include/scenario/monopod/World.h"
    "/home/capstone/Documents/Repos/SIM/src/monopod/include/scenario/monopod/Model.h"
    "/home/capstone/Documents/Repos/SIM/src/monopod/include/scenario/monopod/Joint.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xMonopodx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ScenarioMonopod" TYPE FILE FILES "/home/capstone/Documents/Repos/SIM/ScenarioMonopodConfigVersion.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xMonopodx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ScenarioMonopod" TYPE FILE RENAME "ScenarioMonopodConfig.cmake" FILES "/home/capstone/Documents/Repos/SIM/src/monopod/CMakeFiles/ScenarioMonopodConfig.cmake.install")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xMonopodx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ScenarioMonopod/ScenarioMonopodTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ScenarioMonopod/ScenarioMonopodTargets.cmake"
         "/home/capstone/Documents/Repos/SIM/src/monopod/CMakeFiles/Export/lib/cmake/ScenarioMonopod/ScenarioMonopodTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ScenarioMonopod/ScenarioMonopodTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ScenarioMonopod/ScenarioMonopodTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ScenarioMonopod" TYPE FILE FILES "/home/capstone/Documents/Repos/SIM/src/monopod/CMakeFiles/Export/lib/cmake/ScenarioMonopod/ScenarioMonopodTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ScenarioMonopod" TYPE FILE FILES "/home/capstone/Documents/Repos/SIM/src/monopod/CMakeFiles/Export/lib/cmake/ScenarioMonopod/ScenarioMonopodTargets-release.cmake")
  endif()
endif()

