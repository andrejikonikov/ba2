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
CMAKE_SOURCE_DIR = /home/andrej/studyspace/BA/viewer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andrej/studyspace/BA/viewer/build

# Include any dependencies generated for this target.
include CMakeFiles/Viewer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Viewer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Viewer.dir/flags.make

CMakeFiles/Viewer.dir/src/viewer.cpp.o: CMakeFiles/Viewer.dir/flags.make
CMakeFiles/Viewer.dir/src/viewer.cpp.o: ../src/viewer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/andrej/studyspace/BA/viewer/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Viewer.dir/src/viewer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Viewer.dir/src/viewer.cpp.o -c /home/andrej/studyspace/BA/viewer/src/viewer.cpp

CMakeFiles/Viewer.dir/src/viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Viewer.dir/src/viewer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/andrej/studyspace/BA/viewer/src/viewer.cpp > CMakeFiles/Viewer.dir/src/viewer.cpp.i

CMakeFiles/Viewer.dir/src/viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Viewer.dir/src/viewer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/andrej/studyspace/BA/viewer/src/viewer.cpp -o CMakeFiles/Viewer.dir/src/viewer.cpp.s

CMakeFiles/Viewer.dir/src/viewer.cpp.o.requires:
.PHONY : CMakeFiles/Viewer.dir/src/viewer.cpp.o.requires

CMakeFiles/Viewer.dir/src/viewer.cpp.o.provides: CMakeFiles/Viewer.dir/src/viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/Viewer.dir/build.make CMakeFiles/Viewer.dir/src/viewer.cpp.o.provides.build
.PHONY : CMakeFiles/Viewer.dir/src/viewer.cpp.o.provides

CMakeFiles/Viewer.dir/src/viewer.cpp.o.provides.build: CMakeFiles/Viewer.dir/src/viewer.cpp.o

# Object files for target Viewer
Viewer_OBJECTS = \
"CMakeFiles/Viewer.dir/src/viewer.cpp.o"

# External object files for target Viewer
Viewer_EXTERNAL_OBJECTS =

Viewer: CMakeFiles/Viewer.dir/src/viewer.cpp.o
Viewer: CMakeFiles/Viewer.dir/build.make
Viewer: /usr/lib/x86_64-linux-gnu/libboost_system.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
Viewer: /usr/lib/x86_64-linux-gnu/libpthread.so
Viewer: /usr/local/lib/libpcl_common.so
Viewer: /usr/local/lib/libpcl_octree.so
Viewer: /usr/lib/libOpenNI.so
Viewer: /usr/lib/libOpenNI2.so
Viewer: /usr/local/lib/libpcl_io.so
Viewer: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
Viewer: /usr/local/lib/libpcl_kdtree.so
Viewer: /usr/local/lib/libpcl_search.so
Viewer: /usr/local/lib/libpcl_sample_consensus.so
Viewer: /usr/local/lib/libpcl_filters.so
Viewer: /usr/local/lib/libpcl_features.so
Viewer: /usr/local/lib/libpcl_ml.so
Viewer: /usr/local/lib/libpcl_segmentation.so
Viewer: /usr/lib/x86_64-linux-gnu/libqhull.so
Viewer: /usr/local/lib/libpcl_surface.so
Viewer: /usr/local/lib/libpcl_registration.so
Viewer: /usr/local/lib/libpcl_recognition.so
Viewer: /usr/local/lib/libpcl_keypoints.so
Viewer: /usr/local/lib/libpcl_visualization.so
Viewer: /usr/local/lib/libpcl_tracking.so
Viewer: /usr/local/lib/libpcl_stereo.so
Viewer: /usr/local/lib/libpcl_outofcore.so
Viewer: /usr/local/lib/libpcl_people.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_system.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
Viewer: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
Viewer: /usr/lib/x86_64-linux-gnu/libpthread.so
Viewer: /usr/lib/x86_64-linux-gnu/libqhull.so
Viewer: /usr/lib/libOpenNI.so
Viewer: /usr/lib/libOpenNI2.so
Viewer: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
Viewer: /usr/lib/libvtkGenericFiltering.so.5.8.0
Viewer: /usr/lib/libvtkGeovis.so.5.8.0
Viewer: /usr/lib/libvtkCharts.so.5.8.0
Viewer: /usr/local/lib/libpcl_common.so
Viewer: /usr/local/lib/libpcl_octree.so
Viewer: /usr/local/lib/libpcl_io.so
Viewer: /usr/local/lib/libpcl_kdtree.so
Viewer: /usr/local/lib/libpcl_search.so
Viewer: /usr/local/lib/libpcl_sample_consensus.so
Viewer: /usr/local/lib/libpcl_filters.so
Viewer: /usr/local/lib/libpcl_features.so
Viewer: /usr/local/lib/libpcl_ml.so
Viewer: /usr/local/lib/libpcl_segmentation.so
Viewer: /usr/local/lib/libpcl_surface.so
Viewer: /usr/local/lib/libpcl_registration.so
Viewer: /usr/local/lib/libpcl_recognition.so
Viewer: /usr/local/lib/libpcl_keypoints.so
Viewer: /usr/local/lib/libpcl_visualization.so
Viewer: /usr/local/lib/libpcl_tracking.so
Viewer: /usr/local/lib/libpcl_stereo.so
Viewer: /usr/local/lib/libpcl_outofcore.so
Viewer: /usr/local/lib/libpcl_people.so
Viewer: /usr/lib/libvtkViews.so.5.8.0
Viewer: /usr/lib/libvtkInfovis.so.5.8.0
Viewer: /usr/lib/libvtkWidgets.so.5.8.0
Viewer: /usr/lib/libvtkVolumeRendering.so.5.8.0
Viewer: /usr/lib/libvtkHybrid.so.5.8.0
Viewer: /usr/lib/libvtkParallel.so.5.8.0
Viewer: /usr/lib/libvtkRendering.so.5.8.0
Viewer: /usr/lib/libvtkImaging.so.5.8.0
Viewer: /usr/lib/libvtkGraphics.so.5.8.0
Viewer: /usr/lib/libvtkIO.so.5.8.0
Viewer: /usr/lib/libvtkFiltering.so.5.8.0
Viewer: /usr/lib/libvtkCommon.so.5.8.0
Viewer: /usr/lib/libvtksys.so.5.8.0
Viewer: CMakeFiles/Viewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Viewer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Viewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Viewer.dir/build: Viewer
.PHONY : CMakeFiles/Viewer.dir/build

CMakeFiles/Viewer.dir/requires: CMakeFiles/Viewer.dir/src/viewer.cpp.o.requires
.PHONY : CMakeFiles/Viewer.dir/requires

CMakeFiles/Viewer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Viewer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Viewer.dir/clean

CMakeFiles/Viewer.dir/depend:
	cd /home/andrej/studyspace/BA/viewer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrej/studyspace/BA/viewer /home/andrej/studyspace/BA/viewer /home/andrej/studyspace/BA/viewer/build /home/andrej/studyspace/BA/viewer/build /home/andrej/studyspace/BA/viewer/build/CMakeFiles/Viewer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Viewer.dir/depend
