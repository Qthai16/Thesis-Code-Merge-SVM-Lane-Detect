# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect/build

# Include any dependencies generated for this target.
include CMakeFiles/Control.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Control.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Control.dir/flags.make

CMakeFiles/Control.dir/src/control.cpp.o: CMakeFiles/Control.dir/flags.make
CMakeFiles/Control.dir/src/control.cpp.o: ../src/control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Control.dir/src/control.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Control.dir/src/control.cpp.o -c /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect/src/control.cpp

CMakeFiles/Control.dir/src/control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Control.dir/src/control.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect/src/control.cpp > CMakeFiles/Control.dir/src/control.cpp.i

CMakeFiles/Control.dir/src/control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Control.dir/src/control.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect/src/control.cpp -o CMakeFiles/Control.dir/src/control.cpp.s

CMakeFiles/Control.dir/src/control.cpp.o.requires:

.PHONY : CMakeFiles/Control.dir/src/control.cpp.o.requires

CMakeFiles/Control.dir/src/control.cpp.o.provides: CMakeFiles/Control.dir/src/control.cpp.o.requires
	$(MAKE) -f CMakeFiles/Control.dir/build.make CMakeFiles/Control.dir/src/control.cpp.o.provides.build
.PHONY : CMakeFiles/Control.dir/src/control.cpp.o.provides

CMakeFiles/Control.dir/src/control.cpp.o.provides.build: CMakeFiles/Control.dir/src/control.cpp.o


# Object files for target Control
Control_OBJECTS = \
"CMakeFiles/Control.dir/src/control.cpp.o"

# External object files for target Control
Control_EXTERNAL_OBJECTS =

libControl.a: CMakeFiles/Control.dir/src/control.cpp.o
libControl.a: CMakeFiles/Control.dir/build.make
libControl.a: CMakeFiles/Control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libControl.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Control.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Control.dir/build: libControl.a

.PHONY : CMakeFiles/Control.dir/build

CMakeFiles/Control.dir/requires: CMakeFiles/Control.dir/src/control.cpp.o.requires

.PHONY : CMakeFiles/Control.dir/requires

CMakeFiles/Control.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Control.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Control.dir/clean

CMakeFiles/Control.dir/depend:
	cd /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect/build /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect/build /home/qthai/Downloads/Thesis-Code-Merge-SVM-Lane-Detect/build/CMakeFiles/Control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Control.dir/depend

