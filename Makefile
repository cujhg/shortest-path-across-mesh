# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/clemens/cgal_programs/shortest-path-across-mesh

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/clemens/cgal_programs/shortest-path-across-mesh

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/clemens/cgal_programs/shortest-path-across-mesh/CMakeFiles /home/clemens/cgal_programs/shortest-path-across-mesh/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/clemens/cgal_programs/shortest-path-across-mesh/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named cgal_shortest_path

# Build rule for target.
cgal_shortest_path: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 cgal_shortest_path
.PHONY : cgal_shortest_path

# fast build rule for target.
cgal_shortest_path/fast:
	$(MAKE) -f CMakeFiles/cgal_shortest_path.dir/build.make CMakeFiles/cgal_shortest_path.dir/build
.PHONY : cgal_shortest_path/fast

cgal_shortest_path.o: cgal_shortest_path.cpp.o

.PHONY : cgal_shortest_path.o

# target to build an object file
cgal_shortest_path.cpp.o:
	$(MAKE) -f CMakeFiles/cgal_shortest_path.dir/build.make CMakeFiles/cgal_shortest_path.dir/cgal_shortest_path.cpp.o
.PHONY : cgal_shortest_path.cpp.o

cgal_shortest_path.i: cgal_shortest_path.cpp.i

.PHONY : cgal_shortest_path.i

# target to preprocess a source file
cgal_shortest_path.cpp.i:
	$(MAKE) -f CMakeFiles/cgal_shortest_path.dir/build.make CMakeFiles/cgal_shortest_path.dir/cgal_shortest_path.cpp.i
.PHONY : cgal_shortest_path.cpp.i

cgal_shortest_path.s: cgal_shortest_path.cpp.s

.PHONY : cgal_shortest_path.s

# target to generate assembly for a file
cgal_shortest_path.cpp.s:
	$(MAKE) -f CMakeFiles/cgal_shortest_path.dir/build.make CMakeFiles/cgal_shortest_path.dir/cgal_shortest_path.cpp.s
.PHONY : cgal_shortest_path.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... cgal_shortest_path"
	@echo "... cgal_shortest_path.o"
	@echo "... cgal_shortest_path.i"
	@echo "... cgal_shortest_path.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

