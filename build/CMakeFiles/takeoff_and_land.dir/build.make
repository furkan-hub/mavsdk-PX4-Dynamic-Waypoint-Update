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
CMAKE_SOURCE_DIR = /home/furkan/mavsdk-takeoff_and_land

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/furkan/mavsdk-takeoff_and_land/build

# Include any dependencies generated for this target.
include CMakeFiles/takeoff_and_land.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/takeoff_and_land.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/takeoff_and_land.dir/flags.make

CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.o: CMakeFiles/takeoff_and_land.dir/flags.make
CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.o: ../takeoff_and_land.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/furkan/mavsdk-takeoff_and_land/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.o -c /home/furkan/mavsdk-takeoff_and_land/takeoff_and_land.cpp

CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/furkan/mavsdk-takeoff_and_land/takeoff_and_land.cpp > CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.i

CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/furkan/mavsdk-takeoff_and_land/takeoff_and_land.cpp -o CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.s

CMakeFiles/takeoff_and_land.dir/coordinates.cpp.o: CMakeFiles/takeoff_and_land.dir/flags.make
CMakeFiles/takeoff_and_land.dir/coordinates.cpp.o: ../coordinates.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/furkan/mavsdk-takeoff_and_land/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/takeoff_and_land.dir/coordinates.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/takeoff_and_land.dir/coordinates.cpp.o -c /home/furkan/mavsdk-takeoff_and_land/coordinates.cpp

CMakeFiles/takeoff_and_land.dir/coordinates.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/takeoff_and_land.dir/coordinates.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/furkan/mavsdk-takeoff_and_land/coordinates.cpp > CMakeFiles/takeoff_and_land.dir/coordinates.cpp.i

CMakeFiles/takeoff_and_land.dir/coordinates.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/takeoff_and_land.dir/coordinates.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/furkan/mavsdk-takeoff_and_land/coordinates.cpp -o CMakeFiles/takeoff_and_land.dir/coordinates.cpp.s

# Object files for target takeoff_and_land
takeoff_and_land_OBJECTS = \
"CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.o" \
"CMakeFiles/takeoff_and_land.dir/coordinates.cpp.o"

# External object files for target takeoff_and_land
takeoff_and_land_EXTERNAL_OBJECTS =

takeoff_and_land: CMakeFiles/takeoff_and_land.dir/takeoff_and_land.cpp.o
takeoff_and_land: CMakeFiles/takeoff_and_land.dir/coordinates.cpp.o
takeoff_and_land: CMakeFiles/takeoff_and_land.dir/build.make
takeoff_and_land: /usr/local/lib/libmavsdk.so.2.12.2
takeoff_and_land: CMakeFiles/takeoff_and_land.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/furkan/mavsdk-takeoff_and_land/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable takeoff_and_land"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/takeoff_and_land.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/takeoff_and_land.dir/build: takeoff_and_land

.PHONY : CMakeFiles/takeoff_and_land.dir/build

CMakeFiles/takeoff_and_land.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/takeoff_and_land.dir/cmake_clean.cmake
.PHONY : CMakeFiles/takeoff_and_land.dir/clean

CMakeFiles/takeoff_and_land.dir/depend:
	cd /home/furkan/mavsdk-takeoff_and_land/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/furkan/mavsdk-takeoff_and_land /home/furkan/mavsdk-takeoff_and_land /home/furkan/mavsdk-takeoff_and_land/build /home/furkan/mavsdk-takeoff_and_land/build /home/furkan/mavsdk-takeoff_and_land/build/CMakeFiles/takeoff_and_land.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/takeoff_and_land.dir/depend

