# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/vagrant/dev/kf/jacobian

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vagrant/dev/kf/jacobian/build

# Include any dependencies generated for this target.
include CMakeFiles/jacobian.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/jacobian.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/jacobian.dir/flags.make

CMakeFiles/jacobian.dir/jacobian.cpp.o: CMakeFiles/jacobian.dir/flags.make
CMakeFiles/jacobian.dir/jacobian.cpp.o: ../jacobian.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vagrant/dev/kf/jacobian/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/jacobian.dir/jacobian.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/jacobian.dir/jacobian.cpp.o -c /home/vagrant/dev/kf/jacobian/jacobian.cpp

CMakeFiles/jacobian.dir/jacobian.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/jacobian.dir/jacobian.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vagrant/dev/kf/jacobian/jacobian.cpp > CMakeFiles/jacobian.dir/jacobian.cpp.i

CMakeFiles/jacobian.dir/jacobian.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/jacobian.dir/jacobian.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vagrant/dev/kf/jacobian/jacobian.cpp -o CMakeFiles/jacobian.dir/jacobian.cpp.s

CMakeFiles/jacobian.dir/jacobian.cpp.o.requires:

.PHONY : CMakeFiles/jacobian.dir/jacobian.cpp.o.requires

CMakeFiles/jacobian.dir/jacobian.cpp.o.provides: CMakeFiles/jacobian.dir/jacobian.cpp.o.requires
	$(MAKE) -f CMakeFiles/jacobian.dir/build.make CMakeFiles/jacobian.dir/jacobian.cpp.o.provides.build
.PHONY : CMakeFiles/jacobian.dir/jacobian.cpp.o.provides

CMakeFiles/jacobian.dir/jacobian.cpp.o.provides.build: CMakeFiles/jacobian.dir/jacobian.cpp.o


# Object files for target jacobian
jacobian_OBJECTS = \
"CMakeFiles/jacobian.dir/jacobian.cpp.o"

# External object files for target jacobian
jacobian_EXTERNAL_OBJECTS =

jacobian: CMakeFiles/jacobian.dir/jacobian.cpp.o
jacobian: CMakeFiles/jacobian.dir/build.make
jacobian: CMakeFiles/jacobian.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vagrant/dev/kf/jacobian/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable jacobian"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/jacobian.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/jacobian.dir/build: jacobian

.PHONY : CMakeFiles/jacobian.dir/build

CMakeFiles/jacobian.dir/requires: CMakeFiles/jacobian.dir/jacobian.cpp.o.requires

.PHONY : CMakeFiles/jacobian.dir/requires

CMakeFiles/jacobian.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/jacobian.dir/cmake_clean.cmake
.PHONY : CMakeFiles/jacobian.dir/clean

CMakeFiles/jacobian.dir/depend:
	cd /home/vagrant/dev/kf/jacobian/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vagrant/dev/kf/jacobian /home/vagrant/dev/kf/jacobian /home/vagrant/dev/kf/jacobian/build /home/vagrant/dev/kf/jacobian/build /home/vagrant/dev/kf/jacobian/build/CMakeFiles/jacobian.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/jacobian.dir/depend

