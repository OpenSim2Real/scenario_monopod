# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/capstone/Documents/Repos/SIM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/capstone/Documents/Repos/SIM

# Include any dependencies generated for this target.
include src/monopod/CMakeFiles/ScenarioMonopod.dir/depend.make

# Include the progress variables for this target.
include src/monopod/CMakeFiles/ScenarioMonopod.dir/progress.make

# Include the compile flags for this target's objects.
include src/monopod/CMakeFiles/ScenarioMonopod.dir/flags.make

src/monopod/CMakeFiles/ScenarioMonopod.dir/src/World.cpp.o: src/monopod/CMakeFiles/ScenarioMonopod.dir/flags.make
src/monopod/CMakeFiles/ScenarioMonopod.dir/src/World.cpp.o: src/monopod/src/World.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/capstone/Documents/Repos/SIM/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/monopod/CMakeFiles/ScenarioMonopod.dir/src/World.cpp.o"
	cd /home/capstone/Documents/Repos/SIM/src/monopod && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ScenarioMonopod.dir/src/World.cpp.o -c /home/capstone/Documents/Repos/SIM/src/monopod/src/World.cpp

src/monopod/CMakeFiles/ScenarioMonopod.dir/src/World.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ScenarioMonopod.dir/src/World.cpp.i"
	cd /home/capstone/Documents/Repos/SIM/src/monopod && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/capstone/Documents/Repos/SIM/src/monopod/src/World.cpp > CMakeFiles/ScenarioMonopod.dir/src/World.cpp.i

src/monopod/CMakeFiles/ScenarioMonopod.dir/src/World.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ScenarioMonopod.dir/src/World.cpp.s"
	cd /home/capstone/Documents/Repos/SIM/src/monopod && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/capstone/Documents/Repos/SIM/src/monopod/src/World.cpp -o CMakeFiles/ScenarioMonopod.dir/src/World.cpp.s

src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.o: src/monopod/CMakeFiles/ScenarioMonopod.dir/flags.make
src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.o: src/monopod/src/Model.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/capstone/Documents/Repos/SIM/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.o"
	cd /home/capstone/Documents/Repos/SIM/src/monopod && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.o -c /home/capstone/Documents/Repos/SIM/src/monopod/src/Model.cpp

src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.i"
	cd /home/capstone/Documents/Repos/SIM/src/monopod && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/capstone/Documents/Repos/SIM/src/monopod/src/Model.cpp > CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.i

src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.s"
	cd /home/capstone/Documents/Repos/SIM/src/monopod && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/capstone/Documents/Repos/SIM/src/monopod/src/Model.cpp -o CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.s

src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.o: src/monopod/CMakeFiles/ScenarioMonopod.dir/flags.make
src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.o: src/monopod/src/Joint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/capstone/Documents/Repos/SIM/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.o"
	cd /home/capstone/Documents/Repos/SIM/src/monopod && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.o -c /home/capstone/Documents/Repos/SIM/src/monopod/src/Joint.cpp

src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.i"
	cd /home/capstone/Documents/Repos/SIM/src/monopod && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/capstone/Documents/Repos/SIM/src/monopod/src/Joint.cpp > CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.i

src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.s"
	cd /home/capstone/Documents/Repos/SIM/src/monopod && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/capstone/Documents/Repos/SIM/src/monopod/src/Joint.cpp -o CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.s

# Object files for target ScenarioMonopod
ScenarioMonopod_OBJECTS = \
"CMakeFiles/ScenarioMonopod.dir/src/World.cpp.o" \
"CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.o" \
"CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.o"

# External object files for target ScenarioMonopod
ScenarioMonopod_EXTERNAL_OBJECTS =

lib/libScenarioMonopod.so: src/monopod/CMakeFiles/ScenarioMonopod.dir/src/World.cpp.o
lib/libScenarioMonopod.so: src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Model.cpp.o
lib/libScenarioMonopod.so: src/monopod/CMakeFiles/ScenarioMonopod.dir/src/Joint.cpp.o
lib/libScenarioMonopod.so: src/monopod/CMakeFiles/ScenarioMonopod.dir/build.make
lib/libScenarioMonopod.so: /home/capstone/.local/lib/python3.8/site-packages/scenario/lib/libCoreUtils.a
lib/libScenarioMonopod.so: src/monopod/CMakeFiles/ScenarioMonopod.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/capstone/Documents/Repos/SIM/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library ../../lib/libScenarioMonopod.so"
	cd /home/capstone/Documents/Repos/SIM/src/monopod && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ScenarioMonopod.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/monopod/CMakeFiles/ScenarioMonopod.dir/build: lib/libScenarioMonopod.so

.PHONY : src/monopod/CMakeFiles/ScenarioMonopod.dir/build

src/monopod/CMakeFiles/ScenarioMonopod.dir/clean:
	cd /home/capstone/Documents/Repos/SIM/src/monopod && $(CMAKE_COMMAND) -P CMakeFiles/ScenarioMonopod.dir/cmake_clean.cmake
.PHONY : src/monopod/CMakeFiles/ScenarioMonopod.dir/clean

src/monopod/CMakeFiles/ScenarioMonopod.dir/depend:
	cd /home/capstone/Documents/Repos/SIM && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/capstone/Documents/Repos/SIM /home/capstone/Documents/Repos/SIM/src/monopod /home/capstone/Documents/Repos/SIM /home/capstone/Documents/Repos/SIM/src/monopod /home/capstone/Documents/Repos/SIM/src/monopod/CMakeFiles/ScenarioMonopod.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/monopod/CMakeFiles/ScenarioMonopod.dir/depend
