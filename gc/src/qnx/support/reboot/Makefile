all:	rebootserver_ampro400 rebootserver_nova8890

rebootserver_ampro400:	rebootserver.c v86_watchdog.c
	gcc -DARCH=AMPRO400 -o rebootserver_ampro400 rebootserver.c v86_watchdog.c

rebootserver_nova8890:  rebootserver.c v86_watchdog.c
	gcc -DARCH=NOVA8890 -o rebootserver_nova8890 rebootserver.c v86_watchdog.c

clean:
	\rm -f rebootserver_ampro400 rebootserver_nova8890 *.o *~

