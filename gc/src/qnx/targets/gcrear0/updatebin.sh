#!/bin/sh
#
#	Update bin directory on gcrear0 from indicated host
#
#	Must be run on gcrear0 as user "vehicle"
#
set -e
NODE=$1
USER=$2
PROGRAM=$3
if test -z $NODE
then
echo "Usage: updatebin.sh NODE USER [PROGRAM]"
exit 1
fi
if test -z $USER
then
echo "Usage: updatebin.sh NODE USER [PROGRAM]"
exit 1
fi
if test -z $PROGRAM
then
$PROGRAM="*"
fi
#	Load installed files from bin
echo "Loading executable $PROGRAM from node $NODE, user $USER"
SRC="/net/${NODE}/home/$USER/sandbox/gc/src/qnx/common/bin/$PROGRAM"
echo "Source: $SRC"
cp ${SRC} ~/bin
