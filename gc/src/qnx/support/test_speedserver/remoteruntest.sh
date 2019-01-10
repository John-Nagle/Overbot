#
#	Test dummy speed server with dummy test program
#
echo "Starting servers"
watchdog remotestartfile.txt &
sleep 4
echo "Starting test client at real time priority"
nice -n-6 /home/nagle/sandbox/gc/src/qnx/support/test_speedserver/x86/o-g/test_speedserver_g -v gcrear1 SPEEDTEST

