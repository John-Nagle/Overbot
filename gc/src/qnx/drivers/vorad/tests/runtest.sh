#!/bin/sh
echo "Running VORAD server under watchdog"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nagle/sandbox/gc/src/qnx/common/lib
# echo $LD_LIBRARY_PATH
/home/nagle/sandbox/gc/src/qnx/support/watchdog/watchdog  startfile.txt