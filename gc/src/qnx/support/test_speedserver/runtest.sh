#
#	Test dummy speed server with dummy test program
#
echo "Starting servers"
nice -n-6 ~/sandbox/gc/src/qnx/common/bin/watchdog startfile.txt &
sleep 4
echo "Starting test client at real time priority"
pterm nice -n-6 ~/sandbox/gc/src/qnx/support/test_speedserver/x86/o-g/test_speedserver_g -v  SPEEDTEST

