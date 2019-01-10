#
#	Makes a bootable QNX CD, given a QNX build file
#
#	***NEEDS WORK***
#
TARGET=/home/public/bootcd.iso
BUILDFILE=$1
WORKDIR=floppyfiles
BLANKFLOPPYIMAGE=blankfloppy.img
WORKFLOPPYIMAGE=workfloppy.img
MOUNTPOINT=tempmountpoint
mkdir $WORKDIR
rm $WORKDIR/.boot
#
#	Build an initial file system, and create a boot image
#
rm $WORKDIR/.boot
./mkifs -v $BUILDFILE $WORKDIR/.boot
#
#	Build an image of a floppy from that boot image
#
#	Works by copying an image of a blank floppy into a file.
#	Then mounts that file, using our privileged "mountpfs" program which
#	safely mounts dummy file systems.
#
cp $BLANKFLOPPYIMAGE $WORKFLOPPYIMAGE
chmod 644 $WORKFLOPPYIMAGE
mountpfs $WORKFLOPPYIMAGE $MOUNTPOINT
cp $WORKDIR/.boot $MOUNTPOINT/.boot
mountpfs -u $MOUNTPOINT
#
#	Build an image of a bootable CD from that floppy
#	***LAST . is bad
mkdir boot
mkisofs -b $WORKFLOPPYIMAGE -c boot/boot.catalog -o $TARGET .
echo Created $TARGET
