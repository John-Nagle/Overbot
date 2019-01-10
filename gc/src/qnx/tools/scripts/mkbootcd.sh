#
#	Makes a bootable QNX CD, given a QNX build file
#
#	Usage: mkbootcd.sh  buildfile.bld targetfile.iso file ...
#
#	J. Nagle
#	Team Overbot
#	July, 2003
#
BUILDFILE=$1
TARGET=$2
CONTENTDIR=$3
#	Get  QNX components from server, if necessary
MKROOTPATH=/common/qnx/build
WORKDIR=tempworkdir
BLANKFLOPPYIMAGE=blankfloppy.img
WORKFLOPPYIMAGE=workfloppy.img
MOUNTPOINT=tempmountpoint
#	Check arguments
if [[ ! -n $3 ]]
then echo "usage: mkbootcd.sh buildfile.bld targetfile.iso cdcontentdir"
	return 1
fi
if [[ ! -r $BUILDFILE ]]
then echo Cannot open $BUILDFILE for reading.
	return 1
fi
if [[ ! -r $CONTENTDIR ]]
then echo Cannot open $CONTENTDIR for reading.
	return 1
fi
#	Make working directory in which files for the boot image will be placed.
if [[ -r $WORKDIR ]]
then
	rm -R $WORKDIR										## get rid of old workdir
fi
mkdir $WORKDIR
#
#	Build an initial file system, and create a boot image
#
if ! mkifs -r $MKROOTPATH $BUILDFILE $WORKDIR/.boot
then echo "Unable to build boot image"
	return 1
fi
#
#	Build an image of a floppy from that boot image
#	This is done by mounting a copy of an image of a blank floppy
#	as a file system.
#
cp $BLANKFLOPPYIMAGE $CONTENTDIR/$WORKFLOPPYIMAGE
chmod 644 $CONTENTDIR/$WORKFLOPPYIMAGE
mountpfs -v $WORKFLOPPYIMAGE $MOUNTPOINT
cp $WORKDIR/.boot $MOUNTPOINT/.boot
#	Add code here to copy additional files to boot file system, if needed.
if ! mountpfs -u -v  $MOUNTPOINT
then echo "Unable to mount psuedo-floppy file system. Check that 'mountpfs' is working."
	return 1
fi
#
#	Build an image of a bootable CD from that floppy
echo Adding files from $CONTENTDIR to CD.
if ! mkisofs -b $WORKFLOPPYIMAGE -c boot.catalog -o $TARGET $CONTENTDIR
then echo "Unable to build .iso image"
	return 1
fi
echo Created $TARGET
