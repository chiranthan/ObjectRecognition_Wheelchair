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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/developer/pcl_workspace/curr_ex

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/developer/pcl_workspace/curr_ex

# Include any dependencies generated for this target.
include CMakeFiles/correspondence_grouping.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/correspondence_grouping.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/correspondence_grouping.dir/flags.make

CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o: CMakeFiles/correspondence_grouping.dir/flags.make
CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o: correspondence_grouping.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/developer/pcl_workspace/curr_ex/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o -c /home/developer/pcl_workspace/curr_ex/correspondence_grouping.cpp

CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/developer/pcl_workspace/curr_ex/correspondence_grouping.cpp > CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.i

CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/developer/pcl_workspace/curr_ex/correspondence_grouping.cpp -o CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.s

CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o.requires:
.PHONY : CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o.requires

CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o.provides: CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o.requires
	$(MAKE) -f CMakeFiles/correspondence_grouping.dir/build.make CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o.provides.build
.PHONY : CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o.provides

CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o.provides.build: CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o

# Object files for target correspondence_grouping
correspondence_grouping_OBJECTS = \
"CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o"

# External object files for target correspondence_grouping
correspondence_grouping_EXTERNAL_OBJECTS =

correspondence_grouping: CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o
correspondence_grouping: /usr/lib/libboost_system-mt.so
correspondence_grouping: /usr/lib/libboost_filesystem-mt.so
correspondence_grouping: /usr/lib/libboost_thread-mt.so
correspondence_grouping: /usr/lib/libboost_date_time-mt.so
correspondence_grouping: /usr/lib/libboost_iostreams-mt.so
correspondence_grouping: /usr/lib/libboost_serialization-mt.so
correspondence_grouping: /usr/lib/libpcl_common.so
correspondence_grouping: /usr/lib/libflann_cpp_s.a
correspondence_grouping: /usr/lib/libpcl_kdtree.so
correspondence_grouping: /usr/lib/libpcl_octree.so
correspondence_grouping: /usr/lib/libpcl_search.so
correspondence_grouping: /usr/lib/libOpenNI.so
correspondence_grouping: /usr/lib/libvtkCommon.so.5.8.0
correspondence_grouping: /usr/lib/libvtkRendering.so.5.8.0
correspondence_grouping: /usr/lib/libvtkHybrid.so.5.8.0
correspondence_grouping: /usr/lib/libvtkCharts.so.5.8.0
correspondence_grouping: /usr/lib/libpcl_io.so
correspondence_grouping: /usr/lib/libpcl_sample_consensus.so
correspondence_grouping: /usr/lib/libpcl_filters.so
correspondence_grouping: /usr/lib/libpcl_visualization.so
correspondence_grouping: /usr/lib/libpcl_outofcore.so
correspondence_grouping: /usr/lib/libpcl_features.so
correspondence_grouping: /usr/lib/libpcl_segmentation.so
correspondence_grouping: /usr/lib/libpcl_people.so
correspondence_grouping: /usr/lib/libpcl_registration.so
correspondence_grouping: /usr/lib/libpcl_recognition.so
correspondence_grouping: /usr/lib/libpcl_keypoints.so
correspondence_grouping: /usr/lib/libqhull.so
correspondence_grouping: /usr/lib/libpcl_surface.so
correspondence_grouping: /usr/lib/libpcl_tracking.so
correspondence_grouping: /usr/lib/libpcl_apps.so
correspondence_grouping: /usr/lib/libboost_system-mt.so
correspondence_grouping: /usr/lib/libboost_filesystem-mt.so
correspondence_grouping: /usr/lib/libboost_thread-mt.so
correspondence_grouping: /usr/lib/libboost_date_time-mt.so
correspondence_grouping: /usr/lib/libboost_iostreams-mt.so
correspondence_grouping: /usr/lib/libboost_serialization-mt.so
correspondence_grouping: /usr/lib/libqhull.so
correspondence_grouping: /usr/lib/libOpenNI.so
correspondence_grouping: /usr/lib/libflann_cpp_s.a
correspondence_grouping: /usr/lib/libvtkCommon.so.5.8.0
correspondence_grouping: /usr/lib/libvtkRendering.so.5.8.0
correspondence_grouping: /usr/lib/libvtkHybrid.so.5.8.0
correspondence_grouping: /usr/lib/libvtkCharts.so.5.8.0
correspondence_grouping: /usr/lib/libpcl_common.so
correspondence_grouping: /usr/lib/libpcl_kdtree.so
correspondence_grouping: /usr/lib/libpcl_octree.so
correspondence_grouping: /usr/lib/libpcl_search.so
correspondence_grouping: /usr/lib/libpcl_io.so
correspondence_grouping: /usr/lib/libpcl_sample_consensus.so
correspondence_grouping: /usr/lib/libpcl_filters.so
correspondence_grouping: /usr/lib/libpcl_visualization.so
correspondence_grouping: /usr/lib/libpcl_outofcore.so
correspondence_grouping: /usr/lib/libpcl_features.so
correspondence_grouping: /usr/lib/libpcl_segmentation.so
correspondence_grouping: /usr/lib/libpcl_people.so
correspondence_grouping: /usr/lib/libpcl_registration.so
correspondence_grouping: /usr/lib/libpcl_recognition.so
correspondence_grouping: /usr/lib/libpcl_keypoints.so
correspondence_grouping: /usr/lib/libpcl_surface.so
correspondence_grouping: /usr/lib/libpcl_tracking.so
correspondence_grouping: /usr/lib/libpcl_apps.so
correspondence_grouping: /usr/lib/libvtkViews.so.5.8.0
correspondence_grouping: /usr/lib/libvtkInfovis.so.5.8.0
correspondence_grouping: /usr/lib/libvtkWidgets.so.5.8.0
correspondence_grouping: /usr/lib/libvtkHybrid.so.5.8.0
correspondence_grouping: /usr/lib/libvtkParallel.so.5.8.0
correspondence_grouping: /usr/lib/libvtkVolumeRendering.so.5.8.0
correspondence_grouping: /usr/lib/libvtkRendering.so.5.8.0
correspondence_grouping: /usr/lib/libvtkGraphics.so.5.8.0
correspondence_grouping: /usr/lib/libvtkImaging.so.5.8.0
correspondence_grouping: /usr/lib/libvtkIO.so.5.8.0
correspondence_grouping: /usr/lib/libvtkFiltering.so.5.8.0
correspondence_grouping: /usr/lib/libvtkCommon.so.5.8.0
correspondence_grouping: /usr/lib/libvtksys.so.5.8.0
correspondence_grouping: CMakeFiles/correspondence_grouping.dir/build.make
correspondence_grouping: CMakeFiles/correspondence_grouping.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable correspondence_grouping"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/correspondence_grouping.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/correspondence_grouping.dir/build: correspondence_grouping
.PHONY : CMakeFiles/correspondence_grouping.dir/build

CMakeFiles/correspondence_grouping.dir/requires: CMakeFiles/correspondence_grouping.dir/correspondence_grouping.cpp.o.requires
.PHONY : CMakeFiles/correspondence_grouping.dir/requires

CMakeFiles/correspondence_grouping.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/correspondence_grouping.dir/cmake_clean.cmake
.PHONY : CMakeFiles/correspondence_grouping.dir/clean

CMakeFiles/correspondence_grouping.dir/depend:
	cd /home/developer/pcl_workspace/curr_ex && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/developer/pcl_workspace/curr_ex /home/developer/pcl_workspace/curr_ex /home/developer/pcl_workspace/curr_ex /home/developer/pcl_workspace/curr_ex /home/developer/pcl_workspace/curr_ex/CMakeFiles/correspondence_grouping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/correspondence_grouping.dir/depend

