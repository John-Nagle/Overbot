#!/bin/sh
echo "Running road server under watchdog"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nagle/sandbox/gc/src/qnx/common/lib:/home/vehicle/bin
echo $LD_LIBRARY_PATH
/home/nagle/sandbox/gc/src/qnx/support/watchdog/watchdog   startfileveh2.txt