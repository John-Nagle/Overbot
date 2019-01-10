#!/bin/sh
echo "Running road server under watchdog"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nagle/sandbox/gc/src/qnx/common/lib:/home/vehicle/lib
echo $LD_LIBRARY_PATH
/home/nagle/sandbox/gc/src/qnx/support/watchdog/watchdog   startfile3.txt