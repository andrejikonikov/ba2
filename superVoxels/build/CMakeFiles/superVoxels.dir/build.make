# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/andrej/studyspace/BA/superVoxels

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andrej/studyspace/BA/superVoxels/build

# Include any dependencies generated for this target.
include CMakeFiles/superVoxels.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/superVoxels.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/superVoxels.dir/flags.make

CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o: CMakeFiles/superVoxels.dir/flags.make
CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o: ../src/superVoxels.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/andrej/studyspace/BA/superVoxels/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o -c /home/andrej/studyspace/BA/superVoxels/src/superVoxels.cpp

CMakeFiles/superVoxels.dir/src/superVoxels.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/superVoxels.dir/src/superVoxels.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/andrej/studyspace/BA/superVoxels/src/superVoxels.cpp > CMakeFiles/superVoxels.dir/src/superVoxels.cpp.i

CMakeFiles/superVoxels.dir/src/superVoxels.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/superVoxels.dir/src/superVoxels.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/andrej/studyspace/BA/superVoxels/src/superVoxels.cpp -o CMakeFiles/superVoxels.dir/src/superVoxels.cpp.s

CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o.requires:
.PHONY : CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o.requires

CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o.provides: CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o.requires
	$(MAKE) -f CMakeFiles/superVoxels.dir/build.make CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o.provides.build
.PHONY : CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o.provides

CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o.provides.build: CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o

# Object files for target superVoxels
superVoxels_OBJECTS = \
"CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o"

# External object files for target superVoxels
superVoxels_EXTERNAL_OBJECTS =

superVoxels: CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o
superVoxels: CMakeFiles/superVoxels.dir/build.make
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_system.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_thread.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
superVoxels: /usr/lib/x86_64-linux-gnu/libpthread.so
superVoxels: /usr/local/lib/libpcl_common.so
superVoxels: /usr/local/lib/libpcl_octree.so
superVoxels: /usr/lib/libOpenNI.so
superVoxels: /usr/lib/libOpenNI2.so
superVoxels: /usr/local/lib/libpcl_io.so
superVoxels: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
superVoxels: /usr/local/lib/libpcl_kdtree.so
superVoxels: /usr/local/lib/libpcl_search.so
superVoxels: /usr/local/lib/libpcl_sample_consensus.so
superVoxels: /usr/local/lib/libpcl_filters.so
superVoxels: /usr/local/lib/libpcl_features.so
superVoxels: /usr/local/lib/libpcl_ml.so
superVoxels: /usr/local/lib/libpcl_segmentation.so
superVoxels: /usr/lib/x86_64-linux-gnu/libqhull.so
superVoxels: /usr/local/lib/libpcl_surface.so
superVoxels: /usr/local/lib/libpcl_registration.so
superVoxels: /usr/local/lib/libpcl_recognition.so
superVoxels: /usr/local/lib/libpcl_keypoints.so
superVoxels: /usr/local/lib/libpcl_visualization.so
superVoxels: /usr/local/lib/libpcl_tracking.so
superVoxels: /usr/local/lib/libpcl_stereo.so
superVoxels: /usr/local/lib/libpcl_outofcore.so
superVoxels: /usr/local/lib/libpcl_people.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_system.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_thread.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
superVoxels: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
superVoxels: /usr/lib/x86_64-linux-gnu/libpthread.so
superVoxels: /usr/lib/x86_64-linux-gnu/libqhull.so
superVoxels: /usr/lib/libOpenNI.so
superVoxels: /usr/lib/libOpenNI2.so
superVoxels: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
superVoxels: /usr/lib/libvtkGenericFiltering.so.5.8.0
superVoxels: /usr/lib/libvtkGeovis.so.5.8.0
superVoxels: /usr/lib/libvtkCharts.so.5.8.0
superVoxels: /usr/local/lib/libpcl_common.so
superVoxels: /usr/local/lib/libpcl_octree.so
superVoxels: /usr/local/lib/libpcl_io.so
superVoxels: /usr/local/lib/libpcl_kdtree.so
superVoxels: /usr/local/lib/libpcl_search.so
superVoxels: /usr/local/lib/libpcl_sample_consensus.so
superVoxels: /usr/local/lib/libpcl_filters.so
superVoxels: /usr/local/lib/libpcl_features.so
superVoxels: /usr/local/lib/libpcl_ml.so
superVoxels: /usr/local/lib/libpcl_segmentation.so
superVoxels: /usr/local/lib/libpcl_surface.so
superVoxels: /usr/local/lib/libpcl_registration.so
superVoxels: /usr/local/lib/libpcl_recognition.so
superVoxels: /usr/local/lib/libpcl_keypoints.so
superVoxels: /usr/local/lib/libpcl_visualization.so
superVoxels: /usr/local/lib/libpcl_tracking.so
superVoxels: /usr/local/lib/libpcl_stereo.so
superVoxels: /usr/local/lib/libpcl_outofcore.so
superVoxels: /usr/local/lib/libpcl_people.so
superVoxels: /usr/lib/libvtkViews.so.5.8.0
superVoxels: /usr/lib/libvtkInfovis.so.5.8.0
superVoxels: /usr/lib/libvtkWidgets.so.5.8.0
superVoxels: /usr/lib/libvtkVolumeRendering.so.5.8.0
superVoxels: /usr/lib/libvtkHybrid.so.5.8.0
superVoxels: /usr/lib/libvtkParallel.so.5.8.0
superVoxels: /usr/lib/libvtkRendering.so.5.8.0
superVoxels: /usr/lib/libvtkImaging.so.5.8.0
superVoxels: /usr/lib/libvtkGraphics.so.5.8.0
superVoxels: /usr/lib/libvtkIO.so.5.8.0
superVoxels: /usr/lib/libvtkFiltering.so.5.8.0
superVoxels: /usr/lib/libvtkCommon.so.5.8.0
superVoxels: /usr/lib/libvtksys.so.5.8.0
superVoxels: CMakeFiles/superVoxels.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable superVoxels"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/superVoxels.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/superVoxels.dir/build: superVoxels
.PHONY : CMakeFiles/superVoxels.dir/build

CMakeFiles/superVoxels.dir/requires: CMakeFiles/superVoxels.dir/src/superVoxels.cpp.o.requires
.PHONY : CMakeFiles/superVoxels.dir/requires

CMakeFiles/superVoxels.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/superVoxels.dir/cmake_clean.cmake
.PHONY : CMakeFiles/superVoxels.dir/clean

CMakeFiles/superVoxels.dir/depend:
	cd /home/andrej/studyspace/BA/superVoxels/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrej/studyspace/BA/superVoxels /home/andrej/studyspace/BA/superVoxels /home/andrej/studyspace/BA/superVoxels/build /home/andrej/studyspace/BA/superVoxels/build /home/andrej/studyspace/BA/superVoxels/build/CMakeFiles/superVoxels.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/superVoxels.dir/depend

