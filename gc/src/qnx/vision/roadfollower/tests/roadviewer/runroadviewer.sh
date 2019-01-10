#!/bin/sh
#	Runs road viewer. We need this to get to opencv.so
echo "Running road viewer, road server must be running on gcrear0"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/sandbox/gc/src/qnx/common/lib:/home/vehicle/lib
echo $LD_LIBRARY_PATH
~/sandbox/gc/src/qnx/vision/roadfollower/tests/roadviewer/src/gcc_ntox86/viewer
