/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Oct 6 03
 * Overbot
 *
 * Entry point for GPSINS Server
 * Goes into Kalman Filtering loop
 *
 * /bin/gpsins-server --help for usage
 */
#include "thread.h"
#include "getoptions.h"

int verbose         = 0;

//imagine for now
char * ahrsDev     	= "/dev/ser2";
char * ahrsLog 	   	= NULL;

char * fogDev      	= "/dev/ser2";
char * fogLog	   	= NULL;

char * gpsDev      	= "/dev/ser1";
char * gpsLog		= NULL;

char * kfLog 		= NULL;

float dt            = 0.10;

static int
version( void )
{
	cout << "Unknown version" << endl;
	return 0;
}

static int
help( void )
{
	cerr <<
		"Usage: main [options]\n"
		"\n"
		"       -h | --help                     This help\n"
		"       -V | --version                  Display version\n"
		"       -v | --verbose                  Verbose\n"
		"       -a | --ahrs serial_dev          Serial device to use for ahrs\n"
		"		-A | --ahrsLog logFile			Filename to log ahrs\n"
		"       -g | --gps  serial_dev          Serial device to use for gps\n"
		"		-G | --gpsLog logFile			Filename to log gps\n"
		"       -f | --fog  serial_dev          Serial device to use for fog gyro\n"
		"		-F | --fogLog logFile			Filename to log fog\n"
		"       -d | --dt time                  Length of time step\n"
		"		-K | --kfLog logFile			Filename to log Kalman Filter\n"
		"\n"
	     << endl;

	exit(-1);
}


int
main(int argc, char ** argv)
{
	getoptions( &argc, &argv,
			 "h|?|help&",           help,
			 "V|version&",          version,
			 "v|verbose+",          &verbose,
			 "a|ahrs=s",            &ahrsDev,
			 "A|ahrsLog=s",			&ahrsLog,
			 "g|gps=s",             &gpsDev,
			 "G|gpsLog=s",			&gpsLog,
			 "f|fog=s",             &fogDev,
			 "F|fogLog=s",			&fogLog,
			 "d|dt=f",              &dt,
			 "K|kfLog=s",			&kfLog,
			 0
		    );

	Kalman_Filter kf(ahrsDev,ahrsLog, 
					 fogDev, fogLog,
					 gpsDev, gpsLog,
					 kfLog
					 );

	pthread_t server_thread_id;
	pthread_t meter_thread_id;

	//spin server-thread
	pthread_create(&server_thread_id, 0, server_thread,(void*)&kf);

	//spin meter-thread
	pthread_create(&meter_thread_id, 0, meter_thread, (void*)&kf);

	printf("Starting Kalman-Filter loop\n");

	//go into kalman filtering loop by doing select on the three devices
	//consistently
	while (true) {
		kf.step();
		//TODO
		//send heart beat to watchdog
	}

	return 0;
}
