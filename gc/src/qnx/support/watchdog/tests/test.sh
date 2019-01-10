#!/bin/sh
#
#	Shell file for running watchdog test.
#
rm /tmp/watchdogtest
cp watchdogtest /tmp
../watchdog  startfile.txt