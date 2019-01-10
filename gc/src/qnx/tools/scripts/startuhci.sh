#
#	Start UHCI driver to allow joystick access
#
#	Must run as root.
/sbin/devu-uhci &
/sbin/io-hid &
mount -Tio-hid devh-usb.so &
#	Need proper WAITFOR here.
sleep 3
chmod 666 /dev/io-hid/io-hid