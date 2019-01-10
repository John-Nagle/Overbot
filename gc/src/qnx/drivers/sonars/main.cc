/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Nov 4 03
 * Overbot
 *
 * Entry point for Sonar Server
 * Keeps polling 
 *
 * sonars --help for usage
 */

#include <stdio.h>
#include <iostream>
#include <getoptions.h>
#include <srf08.h>

using namespace std;

int verbose = 0;

char * dev = "/dev/ser1";
 
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
		"       -d | --dev serial_dev           Serial device to use for SONAR server\n"
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
			 "d|dev=s",				&dev,
			 0
		    );

	srf08_init(dev);
//	srf08_select_unit(SRF08_UNIT_5);
//	srf08_change_i2c_address(SRF08_UNIT_1);

	//keep polling for data
	
	while (true) {
		
		int num_dists = 6;
		unsigned int dists[num_dists];
		
		dists[0] = srf08_ping(SRF08_UNIT_0);
		dists[1] = srf08_ping(SRF08_UNIT_1);
		dists[2] = srf08_ping(SRF08_UNIT_2);
		dists[3] = srf08_ping(SRF08_UNIT_3);
		dists[4] = srf08_ping(SRF08_UNIT_4);
		dists[5] = srf08_ping(SRF08_UNIT_5);

		for (int i = 0; i < num_dists; i++) {
			if (dists[i] < 1000) { //sanity check
				printf(" %d ", dists[i]);
			}
			else {
				printf(" INV ");
			}
		}
		printf("\n");
	}

	//TODO
	//send heart beat to watchdog
	
	
//	srf08_select_unit(SRF08_UNIT_0);
//	srf08_change_i2c_address(SRF08_UNIT_1);
/*
	int counts = 0;
	while (1) {
		//i2c_start();
		//i2c_transmit(10);
		//i2c_stop();
		srf08_ping(SRF08_UNIT_0);
		printf("counts: %d\n", counts++);
		
	}
*/
	
	return 0;
}
