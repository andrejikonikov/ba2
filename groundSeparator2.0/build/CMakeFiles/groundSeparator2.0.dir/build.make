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
CMAKE_SOURCE_DIR = /home/andrej/studyspace/BA/groundSeparator2.0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andrej/studyspace/BA/groundSeparator2.0/build

# Include any dependencies generated for this target.
include CMakeFiles/groundSeparator2.0.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/groundSeparator2.0.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/groundSeparator2.0.dir/flags.make

CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o: CMakeFiles/groundSeparator2.0.dir/flags.make
CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o: ../src/groundSeparator2.0.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/andrej/studyspace/BA/groundSeparator2.0/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o -c /home/andrej/studyspace/BA/groundSeparator2.0/src/groundSeparator2.0.cpp

CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/andrej/studyspace/BA/groundSeparator2.0/src/groundSeparator2.0.cpp > CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.i

CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/andrej/studyspace/BA/groundSeparator2.0/src/groundSeparator2.0.cpp -o CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.s

CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o.requires:
.PHONY : CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o.requires

CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o.provides: CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o.requires
	$(MAKE) -f CMakeFiles/groundSeparator2.0.dir/build.make CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o.provides.build
.PHONY : CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o.provides

CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o.provides.build: CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o

# Object files for target groundSeparator2.0
groundSeparator2_0_OBJECTS = \
"CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o"

# External object files for target groundSeparator2.0
groundSeparator2_0_EXTERNAL_OBJECTS =

groundSeparator2.0: CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o
groundSeparator2.0: CMakeFiles/groundSeparator2.0.dir/build.make
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_system.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_thread.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libpthread.so
groundSeparator2.0: /usr/local/lib/libpcl_common.so
groundSeparator2.0: /usr/local/lib/libpcl_octree.so
groundSeparator2.0: /usr/lib/libOpenNI.so
groundSeparator2.0: /usr/lib/libOpenNI2.so
groundSeparator2.0: /usr/local/lib/libpcl_io.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
groundSeparator2.0: /usr/local/lib/libpcl_kdtree.so
groundSeparator2.0: /usr/local/lib/libpcl_search.so
groundSeparator2.0: /usr/local/lib/libpcl_sample_consensus.so
groundSeparator2.0: /usr/local/lib/libpcl_filters.so
groundSeparator2.0: /usr/local/lib/libpcl_features.so
groundSeparator2.0: /usr/local/lib/libpcl_ml.so
groundSeparator2.0: /usr/local/lib/libpcl_segmentation.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libqhull.so
groundSeparator2.0: /usr/local/lib/libpcl_surface.so
groundSeparator2.0: /usr/local/lib/libpcl_registration.so
groundSeparator2.0: /usr/local/lib/libpcl_recognition.so
groundSeparator2.0: /usr/local/lib/libpcl_keypoints.so
groundSeparator2.0: /usr/local/lib/libpcl_visualization.so
groundSeparator2.0: /usr/local/lib/libpcl_tracking.so
groundSeparator2.0: /usr/local/lib/libpcl_stereo.so
groundSeparator2.0: /usr/local/lib/libpcl_outofcore.so
groundSeparator2.0: /usr/local/lib/libpcl_people.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_system.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_thread.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libpthread.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libqhull.so
groundSeparator2.0: /usr/lib/libOpenNI.so
groundSeparator2.0: /usr/lib/libOpenNI2.so
groundSeparator2.0: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
groundSeparator2.0: /usr/lib/libvtkGenericFiltering.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkGeovis.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkCharts.so.5.8.0
groundSeparator2.0: /usr/local/lib/libpcl_common.so
groundSeparator2.0: /usr/local/lib/libpcl_octree.so
groundSeparator2.0: /usr/local/lib/libpcl_io.so
groundSeparator2.0: /usr/local/lib/libpcl_kdtree.so
groundSeparator2.0: /usr/local/lib/libpcl_search.so
groundSeparator2.0: /usr/local/lib/libpcl_sample_consensus.so
groundSeparator2.0: /usr/local/lib/libpcl_filters.so
groundSeparator2.0: /usr/local/lib/libpcl_features.so
groundSeparator2.0: /usr/local/lib/libpcl_ml.so
groundSeparator2.0: /usr/local/lib/libpcl_segmentation.so
groundSeparator2.0: /usr/local/lib/libpcl_surface.so
groundSeparator2.0: /usr/local/lib/libpcl_registration.so
groundSeparator2.0: /usr/local/lib/libpcl_recognition.so
groundSeparator2.0: /usr/local/lib/libpcl_keypoints.so
groundSeparator2.0: /usr/local/lib/libpcl_visualization.so
groundSeparator2.0: /usr/local/lib/libpcl_tracking.so
groundSeparator2.0: /usr/local/lib/libpcl_stereo.so
groundSeparator2.0: /usr/local/lib/libpcl_outofcore.so
groundSeparator2.0: /usr/local/lib/libpcl_people.so
groundSeparator2.0: /usr/lib/libvtkViews.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkInfovis.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkWidgets.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkVolumeRendering.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkHybrid.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkParallel.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkRendering.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkImaging.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkGraphics.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkIO.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkFiltering.so.5.8.0
groundSeparator2.0: /usr/lib/libvtkCommon.so.5.8.0
groundSeparator2.0: /usr/lib/libvtksys.so.5.8.0
groundSeparator2.0: CMakeFiles/groundSeparator2.0.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable groundSeparator2.0"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/groundSeparator2.0.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/groundSeparator2.0.dir/build: groundSeparator2.0
.PHONY : CMakeFiles/groundSeparator2.0.dir/build

CMakeFiles/groundSeparator2.0.dir/requires: CMakeFiles/groundSeparator2.0.dir/src/groundSeparator2.0.cpp.o.requires
.PHONY : CMakeFiles/groundSeparator2.0.dir/requires

CMakeFiles/groundSeparator2.0.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/groundSeparator2.0.dir/cmake_clean.cmake
.PHONY : CMakeFiles/groundSeparator2.0.dir/clean

CMakeFiles/groundSeparator2.0.dir/depend:
	cd /home/andrej/studyspace/BA/groundSeparator2.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrej/studyspace/BA/groundSeparator2.0 /home/andrej/studyspace/BA/groundSeparator2.0 /home/andrej/studyspace/BA/groundSeparator2.0/build /home/andrej/studyspace/BA/groundSeparator2.0/build /home/andrej/studyspace/BA/groundSeparator2.0/build/CMakeFiles/groundSeparator2.0.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/groundSeparator2.0.dir/depend

