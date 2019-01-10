#!/bin/sh
#
#	The Overbot start script for gcrear0
#  J. Pierre   9/17/2005
#
#	Run this as user "vehicle" on "gcrear0"
#	Started from rc.vehicle
#  This script should be located in /home/vehicle/bin
#
MYDIR=/home/vehicle

export HOME=$MYDIR
export PATH=$PATH:$MYDIR/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MYDIR/bin



  while true; do
  
    # check if magic file "goactive" exists in /home/vehicle
    if test ! -f $MYDIR/goactive
      then
      echo "startvehicle.sh: Not active, exiting"
      exit 0
    else
      echo "startvehicle.sh: Active, starting watchdog" 
    fi
    
    # wait until other machine comes up
    # waitfor /net/gcrear2 60
      
    # create a log file name
    MYDATE=`date '+%Y%m%d'`
    MYTIME=`date '+%H%M%S'`
    MYLOGFILE="watchdog"$MYDATE"_"$MYTIME".log"

    ### command to run system in autonomous mode
    watchdog $MYDIR/startdrive.txt > $MYDIR/logs/$MYLOGFILE 2>&1
    
    # check exit code of above command, decide to restart or not
    EXIT_STATUS=$?
    if [ $EXIT_STATUS -ne 2 ]
    then
      echo "startvehicle.sh: Watchdog exit status $EXIT_STATUS, restart"
      sleep 2
    else
      echo "startvehicle.sh: Watchdog exit status $EXIT_STATUS, die"
      break
    fi
    
  done
  

exit 0
