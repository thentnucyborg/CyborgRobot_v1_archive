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
CMAKE_SOURCE_DIR = /home/viki/Desktop/speech_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/viki/Desktop/speech_ws/build

# Include any dependencies generated for this target.
include trollnode-kinect_img/CMakeFiles/publishExpression.dir/depend.make

# Include the progress variables for this target.
include trollnode-kinect_img/CMakeFiles/publishExpression.dir/progress.make

# Include the compile flags for this target's objects.
include trollnode-kinect_img/CMakeFiles/publishExpression.dir/flags.make

trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o: trollnode-kinect_img/CMakeFiles/publishExpression.dir/flags.make
trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o: /home/viki/Desktop/speech_ws/src/trollnode-kinect_img/src/publishExpression.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/viki/Desktop/speech_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o"
	cd /home/viki/Desktop/speech_ws/build/trollnode-kinect_img && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o -c /home/viki/Desktop/speech_ws/src/trollnode-kinect_img/src/publishExpression.cpp

trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/publishExpression.dir/src/publishExpression.cpp.i"
	cd /home/viki/Desktop/speech_ws/build/trollnode-kinect_img && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/viki/Desktop/speech_ws/src/trollnode-kinect_img/src/publishExpression.cpp > CMakeFiles/publishExpression.dir/src/publishExpression.cpp.i

trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/publishExpression.dir/src/publishExpression.cpp.s"
	cd /home/viki/Desktop/speech_ws/build/trollnode-kinect_img && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/viki/Desktop/speech_ws/src/trollnode-kinect_img/src/publishExpression.cpp -o CMakeFiles/publishExpression.dir/src/publishExpression.cpp.s

trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o.requires:
.PHONY : trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o.requires

trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o.provides: trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o.requires
	$(MAKE) -f trollnode-kinect_img/CMakeFiles/publishExpression.dir/build.make trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o.provides.build
.PHONY : trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o.provides

trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o.provides.build: trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o

# Object files for target publishExpression
publishExpression_OBJECTS = \
"CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o"

# External object files for target publishExpression
publishExpression_EXTERNAL_OBJECTS =

/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libimage_transport.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libmessage_filters.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/libtinyxml.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libclass_loader.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/libPocoFoundation.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/i386-linux-gnu/libdl.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libroscpp.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/libboost_signals-mt.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/libboost_filesystem-mt.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libxmlrpcpp.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libroslib.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libcv_bridge.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_calib3d.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_contrib.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_core.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_features2d.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_flann.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_gpu.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_highgui.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_imgproc.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_legacy.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_ml.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_nonfree.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_objdetect.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_photo.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_stitching.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_superres.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_video.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libopencv_videostab.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/librosconsole.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/librosconsole_log4cxx.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/librosconsole_backend_interface.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/liblog4cxx.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/libboost_regex-mt.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libroscpp_serialization.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/librostime.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/libboost_date_time-mt.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/libboost_system-mt.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/libboost_thread-mt.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /usr/lib/i386-linux-gnu/libpthread.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libcpp_common.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: /opt/ros/hydro/lib/libconsole_bridge.so
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: trollnode-kinect_img/CMakeFiles/publishExpression.dir/build.make
/home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression: trollnode-kinect_img/CMakeFiles/publishExpression.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression"
	cd /home/viki/Desktop/speech_ws/build/trollnode-kinect_img && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/publishExpression.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
trollnode-kinect_img/CMakeFiles/publishExpression.dir/build: /home/viki/Desktop/speech_ws/devel/lib/trollnode/publishExpression
.PHONY : trollnode-kinect_img/CMakeFiles/publishExpression.dir/build

trollnode-kinect_img/CMakeFiles/publishExpression.dir/requires: trollnode-kinect_img/CMakeFiles/publishExpression.dir/src/publishExpression.cpp.o.requires
.PHONY : trollnode-kinect_img/CMakeFiles/publishExpression.dir/requires

trollnode-kinect_img/CMakeFiles/publishExpression.dir/clean:
	cd /home/viki/Desktop/speech_ws/build/trollnode-kinect_img && $(CMAKE_COMMAND) -P CMakeFiles/publishExpression.dir/cmake_clean.cmake
.PHONY : trollnode-kinect_img/CMakeFiles/publishExpression.dir/clean

trollnode-kinect_img/CMakeFiles/publishExpression.dir/depend:
	cd /home/viki/Desktop/speech_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/viki/Desktop/speech_ws/src /home/viki/Desktop/speech_ws/src/trollnode-kinect_img /home/viki/Desktop/speech_ws/build /home/viki/Desktop/speech_ws/build/trollnode-kinect_img /home/viki/Desktop/speech_ws/build/trollnode-kinect_img/CMakeFiles/publishExpression.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : trollnode-kinect_img/CMakeFiles/publishExpression.dir/depend

