#	Simple, quiet test of map building, reading a canned file with a scan from a fixed position.
#	Runs on latest file in log directory, unless a file is speciifed.
DIR=/tmp/logs
if test $1
then
	LIDARFILE=" -s $1"
else
	LIDARFILE=`ls -t -1 $DIR/lidarscans*.log | head -n 1`
	LIDARFILE="-s $LIDARFILE"
fi
if test $2
then
	GPSFILE="-g $2"
else
	GPSFILE=""
fi
if test $3
then
	WAYPTFILE="-w $3"
else
	WAYPTFILE=""
fi
echo "Reading: $LIDARFILE $GPSFILE $WAYPTFILE"
nice ./x86/o/map  $LIDARFILE $GPSFILE $WAYPTFILE -d /tmp/maplogs 

