# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = D:\CMake\bin\cmake.exe

# The command to remove a file.
RM = D:\CMake\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\build

# Include any dependencies generated for this target.
include CMakeFiles/RayTracing.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/RayTracing.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/RayTracing.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RayTracing.dir/flags.make

CMakeFiles/RayTracing.dir/main.cpp.obj: CMakeFiles/RayTracing.dir/flags.make
CMakeFiles/RayTracing.dir/main.cpp.obj: ../main.cpp
CMakeFiles/RayTracing.dir/main.cpp.obj: CMakeFiles/RayTracing.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RayTracing.dir/main.cpp.obj"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracing.dir/main.cpp.obj -MF CMakeFiles\RayTracing.dir\main.cpp.obj.d -o CMakeFiles\RayTracing.dir\main.cpp.obj -c D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\main.cpp

CMakeFiles/RayTracing.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RayTracing.dir/main.cpp.i"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\main.cpp > CMakeFiles\RayTracing.dir\main.cpp.i

CMakeFiles/RayTracing.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RayTracing.dir/main.cpp.s"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\main.cpp -o CMakeFiles\RayTracing.dir\main.cpp.s

CMakeFiles/RayTracing.dir/Vector.cpp.obj: CMakeFiles/RayTracing.dir/flags.make
CMakeFiles/RayTracing.dir/Vector.cpp.obj: ../Vector.cpp
CMakeFiles/RayTracing.dir/Vector.cpp.obj: CMakeFiles/RayTracing.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/RayTracing.dir/Vector.cpp.obj"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracing.dir/Vector.cpp.obj -MF CMakeFiles\RayTracing.dir\Vector.cpp.obj.d -o CMakeFiles\RayTracing.dir\Vector.cpp.obj -c D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\Vector.cpp

CMakeFiles/RayTracing.dir/Vector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RayTracing.dir/Vector.cpp.i"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\Vector.cpp > CMakeFiles\RayTracing.dir\Vector.cpp.i

CMakeFiles/RayTracing.dir/Vector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RayTracing.dir/Vector.cpp.s"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\Vector.cpp -o CMakeFiles\RayTracing.dir\Vector.cpp.s

CMakeFiles/RayTracing.dir/Scene.cpp.obj: CMakeFiles/RayTracing.dir/flags.make
CMakeFiles/RayTracing.dir/Scene.cpp.obj: ../Scene.cpp
CMakeFiles/RayTracing.dir/Scene.cpp.obj: CMakeFiles/RayTracing.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/RayTracing.dir/Scene.cpp.obj"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracing.dir/Scene.cpp.obj -MF CMakeFiles\RayTracing.dir\Scene.cpp.obj.d -o CMakeFiles\RayTracing.dir\Scene.cpp.obj -c D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\Scene.cpp

CMakeFiles/RayTracing.dir/Scene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RayTracing.dir/Scene.cpp.i"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\Scene.cpp > CMakeFiles\RayTracing.dir\Scene.cpp.i

CMakeFiles/RayTracing.dir/Scene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RayTracing.dir/Scene.cpp.s"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\Scene.cpp -o CMakeFiles\RayTracing.dir\Scene.cpp.s

CMakeFiles/RayTracing.dir/BVH.cpp.obj: CMakeFiles/RayTracing.dir/flags.make
CMakeFiles/RayTracing.dir/BVH.cpp.obj: ../BVH.cpp
CMakeFiles/RayTracing.dir/BVH.cpp.obj: CMakeFiles/RayTracing.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/RayTracing.dir/BVH.cpp.obj"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracing.dir/BVH.cpp.obj -MF CMakeFiles\RayTracing.dir\BVH.cpp.obj.d -o CMakeFiles\RayTracing.dir\BVH.cpp.obj -c D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\BVH.cpp

CMakeFiles/RayTracing.dir/BVH.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RayTracing.dir/BVH.cpp.i"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\BVH.cpp > CMakeFiles\RayTracing.dir\BVH.cpp.i

CMakeFiles/RayTracing.dir/BVH.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RayTracing.dir/BVH.cpp.s"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\BVH.cpp -o CMakeFiles\RayTracing.dir\BVH.cpp.s

CMakeFiles/RayTracing.dir/Renderer.cpp.obj: CMakeFiles/RayTracing.dir/flags.make
CMakeFiles/RayTracing.dir/Renderer.cpp.obj: ../Renderer.cpp
CMakeFiles/RayTracing.dir/Renderer.cpp.obj: CMakeFiles/RayTracing.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/RayTracing.dir/Renderer.cpp.obj"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracing.dir/Renderer.cpp.obj -MF CMakeFiles\RayTracing.dir\Renderer.cpp.obj.d -o CMakeFiles\RayTracing.dir\Renderer.cpp.obj -c D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\Renderer.cpp

CMakeFiles/RayTracing.dir/Renderer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RayTracing.dir/Renderer.cpp.i"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\Renderer.cpp > CMakeFiles\RayTracing.dir\Renderer.cpp.i

CMakeFiles/RayTracing.dir/Renderer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RayTracing.dir/Renderer.cpp.s"
	D:\Code_Environment\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\Renderer.cpp -o CMakeFiles\RayTracing.dir\Renderer.cpp.s

# Object files for target RayTracing
RayTracing_OBJECTS = \
"CMakeFiles/RayTracing.dir/main.cpp.obj" \
"CMakeFiles/RayTracing.dir/Vector.cpp.obj" \
"CMakeFiles/RayTracing.dir/Scene.cpp.obj" \
"CMakeFiles/RayTracing.dir/BVH.cpp.obj" \
"CMakeFiles/RayTracing.dir/Renderer.cpp.obj"

# External object files for target RayTracing
RayTracing_EXTERNAL_OBJECTS =

RayTracing.exe: CMakeFiles/RayTracing.dir/main.cpp.obj
RayTracing.exe: CMakeFiles/RayTracing.dir/Vector.cpp.obj
RayTracing.exe: CMakeFiles/RayTracing.dir/Scene.cpp.obj
RayTracing.exe: CMakeFiles/RayTracing.dir/BVH.cpp.obj
RayTracing.exe: CMakeFiles/RayTracing.dir/Renderer.cpp.obj
RayTracing.exe: CMakeFiles/RayTracing.dir/build.make
RayTracing.exe: CMakeFiles/RayTracing.dir/linklibs.rsp
RayTracing.exe: CMakeFiles/RayTracing.dir/objects1.rsp
RayTracing.exe: CMakeFiles/RayTracing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable RayTracing.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\RayTracing.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RayTracing.dir/build: RayTracing.exe
.PHONY : CMakeFiles/RayTracing.dir/build

CMakeFiles/RayTracing.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\RayTracing.dir\cmake_clean.cmake
.PHONY : CMakeFiles/RayTracing.dir/clean

CMakeFiles/RayTracing.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7 D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7 D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\build D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\build D:\学习\计算机图形学\games101作业\Gams101-Homework\Homework7\Assignment7\Assignment7\build\CMakeFiles\RayTracing.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RayTracing.dir/depend

