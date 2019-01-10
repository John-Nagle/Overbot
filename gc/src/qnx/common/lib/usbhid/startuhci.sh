#
#	Start UHCI driver to allow joystick access
#
#	Must run as root.
/sbin/devu-uhci &
/sbin/io-hid &
waitfor /dev/io-hid
mount -Tio-hid devh-usb.so &
#	Wait for device to come up.
waitfor /dev/io-hid/io-hid
chmod 666 /dev/io-hid/io-hid