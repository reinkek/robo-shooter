# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/newt/workspace/cs225a/OptiTrack

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/newt/workspace/cs225a/OptiTrack/build

# Include any dependencies generated for this target.
include CMakeFiles/exampleCode.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/exampleCode.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/exampleCode.dir/flags.make

CMakeFiles/exampleCode.dir/src/exampleCode.o: CMakeFiles/exampleCode.dir/flags.make
CMakeFiles/exampleCode.dir/src/exampleCode.o: ../src/exampleCode.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/newt/workspace/cs225a/OptiTrack/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/exampleCode.dir/src/exampleCode.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/exampleCode.dir/src/exampleCode.o -c /home/newt/workspace/cs225a/OptiTrack/src/exampleCode.cpp

CMakeFiles/exampleCode.dir/src/exampleCode.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exampleCode.dir/src/exampleCode.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/newt/workspace/cs225a/OptiTrack/src/exampleCode.cpp > CMakeFiles/exampleCode.dir/src/exampleCode.i

CMakeFiles/exampleCode.dir/src/exampleCode.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exampleCode.dir/src/exampleCode.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/newt/workspace/cs225a/OptiTrack/src/exampleCode.cpp -o CMakeFiles/exampleCode.dir/src/exampleCode.s

CMakeFiles/exampleCode.dir/src/exampleCode.o.requires:
.PHONY : CMakeFiles/exampleCode.dir/src/exampleCode.o.requires

CMakeFiles/exampleCode.dir/src/exampleCode.o.provides: CMakeFiles/exampleCode.dir/src/exampleCode.o.requires
	$(MAKE) -f CMakeFiles/exampleCode.dir/build.make CMakeFiles/exampleCode.dir/src/exampleCode.o.provides.build
.PHONY : CMakeFiles/exampleCode.dir/src/exampleCode.o.provides

CMakeFiles/exampleCode.dir/src/exampleCode.o.provides.build: CMakeFiles/exampleCode.dir/src/exampleCode.o

# Object files for target exampleCode
exampleCode_OBJECTS = \
"CMakeFiles/exampleCode.dir/src/exampleCode.o"

# External object files for target exampleCode
exampleCode_EXTERNAL_OBJECTS =

../bin/exampleCode: CMakeFiles/exampleCode.dir/src/exampleCode.o
../bin/exampleCode: CMakeFiles/exampleCode.dir/build.make
../bin/exampleCode: ../lib/libOptiTrack.a
../bin/exampleCode: CMakeFiles/exampleCode.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/exampleCode"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/exampleCode.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/exampleCode.dir/build: ../bin/exampleCode
.PHONY : CMakeFiles/exampleCode.dir/build

CMakeFiles/exampleCode.dir/requires: CMakeFiles/exampleCode.dir/src/exampleCode.o.requires
.PHONY : CMakeFiles/exampleCode.dir/requires

CMakeFiles/exampleCode.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/exampleCode.dir/cmake_clean.cmake
.PHONY : CMakeFiles/exampleCode.dir/clean

CMakeFiles/exampleCode.dir/depend:
	cd /home/newt/workspace/cs225a/OptiTrack/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/newt/workspace/cs225a/OptiTrack /home/newt/workspace/cs225a/OptiTrack /home/newt/workspace/cs225a/OptiTrack/build /home/newt/workspace/cs225a/OptiTrack/build /home/newt/workspace/cs225a/OptiTrack/build/CMakeFiles/exampleCode.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/exampleCode.dir/depend

