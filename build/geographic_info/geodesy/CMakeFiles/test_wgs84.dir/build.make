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
CMAKE_SOURCE_DIR = /home/bart/Documents/Nausbot/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bart/Documents/Nausbot/build

# Include any dependencies generated for this target.
include geographic_info/geodesy/CMakeFiles/test_wgs84.dir/depend.make

# Include the progress variables for this target.
include geographic_info/geodesy/CMakeFiles/test_wgs84.dir/progress.make

# Include the compile flags for this target's objects.
include geographic_info/geodesy/CMakeFiles/test_wgs84.dir/flags.make

geographic_info/geodesy/CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.o: geographic_info/geodesy/CMakeFiles/test_wgs84.dir/flags.make
geographic_info/geodesy/CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.o: /home/bart/Documents/Nausbot/src/geographic_info/geodesy/tests/test_wgs84.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bart/Documents/Nausbot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object geographic_info/geodesy/CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.o"
	cd /home/bart/Documents/Nausbot/build/geographic_info/geodesy && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.o -c /home/bart/Documents/Nausbot/src/geographic_info/geodesy/tests/test_wgs84.cpp

geographic_info/geodesy/CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.i"
	cd /home/bart/Documents/Nausbot/build/geographic_info/geodesy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bart/Documents/Nausbot/src/geographic_info/geodesy/tests/test_wgs84.cpp > CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.i

geographic_info/geodesy/CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.s"
	cd /home/bart/Documents/Nausbot/build/geographic_info/geodesy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bart/Documents/Nausbot/src/geographic_info/geodesy/tests/test_wgs84.cpp -o CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.s

# Object files for target test_wgs84
test_wgs84_OBJECTS = \
"CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.o"

# External object files for target test_wgs84
test_wgs84_EXTERNAL_OBJECTS =

/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: geographic_info/geodesy/CMakeFiles/test_wgs84.dir/tests/test_wgs84.cpp.o
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: geographic_info/geodesy/CMakeFiles/test_wgs84.dir/build.make
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: gtest/lib/libgtest.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/libtf.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/libtf2_ros.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/libactionlib.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/libmessage_filters.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/libroscpp.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/libtf2.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/librosconsole.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/librostime.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /opt/ros/noetic/lib/libcpp_common.so
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84: geographic_info/geodesy/CMakeFiles/test_wgs84.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bart/Documents/Nausbot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84"
	cd /home/bart/Documents/Nausbot/build/geographic_info/geodesy && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_wgs84.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
geographic_info/geodesy/CMakeFiles/test_wgs84.dir/build: /home/bart/Documents/Nausbot/devel/lib/geodesy/test_wgs84

.PHONY : geographic_info/geodesy/CMakeFiles/test_wgs84.dir/build

geographic_info/geodesy/CMakeFiles/test_wgs84.dir/clean:
	cd /home/bart/Documents/Nausbot/build/geographic_info/geodesy && $(CMAKE_COMMAND) -P CMakeFiles/test_wgs84.dir/cmake_clean.cmake
.PHONY : geographic_info/geodesy/CMakeFiles/test_wgs84.dir/clean

geographic_info/geodesy/CMakeFiles/test_wgs84.dir/depend:
	cd /home/bart/Documents/Nausbot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bart/Documents/Nausbot/src /home/bart/Documents/Nausbot/src/geographic_info/geodesy /home/bart/Documents/Nausbot/build /home/bart/Documents/Nausbot/build/geographic_info/geodesy /home/bart/Documents/Nausbot/build/geographic_info/geodesy/CMakeFiles/test_wgs84.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : geographic_info/geodesy/CMakeFiles/test_wgs84.dir/depend

