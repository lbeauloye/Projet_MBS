# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build

# Utility rule file for userfct_clean.

# Include the progress variables for this target.
include CMakeFiles/userfct_clean.dir/progress.make

CMakeFiles/userfct_clean:
	/usr/local/bin/cmake -E chdir /Users/vermeulenlucas/.robotran/mbsysc/MBsysC/cmake_aux/scripts ./userfct_build /Users/vermeulenlucas/.robotran/mbsysc/MBsysC /Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR clean

userfct_clean: CMakeFiles/userfct_clean
userfct_clean: CMakeFiles/userfct_clean.dir/build.make

.PHONY : userfct_clean

# Rule to build all files generated by this target.
CMakeFiles/userfct_clean.dir/build: userfct_clean

.PHONY : CMakeFiles/userfct_clean.dir/build

CMakeFiles/userfct_clean.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/userfct_clean.dir/cmake_clean.cmake
.PHONY : CMakeFiles/userfct_clean.dir/clean

CMakeFiles/userfct_clean.dir/depend:
	cd /Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR /Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR /Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build /Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build /Users/vermeulenlucas/Documents/MBProjects/Simple_manege/workR/build/CMakeFiles/userfct_clean.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/userfct_clean.dir/depend
