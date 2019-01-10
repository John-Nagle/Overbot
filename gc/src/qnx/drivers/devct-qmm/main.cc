/**
 * Khian Hao Lim, Team Overbot, December 2003
 * This is our "driver" for the counter timer board Diamond Systems qmm
 * The Universal Driver that comes with it seems to work ok
 * This is just a resource manager that wraps around it so we can
 * have a non-root user get the counter values of the counters
 * Code is based on DSCQMMCounterWatch.c from the Universal Driver sample source
 * 
 * /dev/devct-qmm will be opened and every read from it
 * will make this program go read and reset the counter values back
 * down to 0
 * 
 * The line read from /dev/devct-qmm will look like
 * counter1-val counter1-overflowed, counter2-val, counter2-overflowed,... counter10-val, counter10-overflowed\n
 * eg
 * 11 0, 22 0, 33 0, 44, 1, 55 0, 66 0, 77 0, 88 0, 99 0, 1234 0\n
 * means counter 1 has value of 11 not overflowed while counter 4 has value of 44 and overflowed
 * 
 * There is no way to figure out how many times any counter overflowed, so whenever you see
 * a overflow, the program reading should read /dev/devct-qmm faster
 * 
 * Usage:
 * (run as root or from rc.local)
 * $ ./devct-qmm
 * 
 * For optimization right now we read only up to LAST_USED_COUNTER which should be a positive number >= 1
 * 
 * To properly read off the right format, do:
 * 
 * char counts[10][10];
 * char overflows[10][10];
 * FILE * f = fopen("/dev/devct-qmm", "r");
 * fscanf(f, "%d %d, %d %d, %d %d, %d %d...", counts[0], overflows[0], ...);
 * count0 = atoi(counts[0]);
 * ...
 * 
 * To use to count freq in a user land program,
 * 
 * Read from /dev/devct-qmm (resets the counters) until newline like above
 * last_time = gettimeofday();
 * 
 * With a fixed period
 * 1) Read /dev/devct-qmm
 *    freq = counts/(gettimeofday - last_time)
 * 2) last-time = gettimeofday();
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include <time.h>
// diamond driver includes
#include "dscud.h"

//includes for resource manager
#include <errno.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/sysmgr.h>
#include <sys/neutrino.h>
#include <sys/procfs.h>
#include <errno.h>
#include <fcntl.h>

//our own includes
#include <stdio.h>
#include <iostream>
#include <getoptions.h>

using namespace std;

//the following needs to be between 1 and 10
#define LAST_USED_COUNTER 4

//appended to the front of error printouts
#define ERROR_PREFIX "QMM Driver ERROR:"

//the following depends on the configuration of the counter timer
//board on the pc104 stack
#define BASE_ADDRESS 0x300
//TODO, experiment with this
#define INT_LEVEL 2
//there are 10 counters on the QMM board
#define QMM_MIN_COUNTER 1
#define QMM_MAX_COUNTER 10

//here we set it to 1 just to test whether the board is present
//it should be 0 in production
#define INITIAL_VALUE 1

//for the counter timer universal driver
BYTE result;    // returned error code
DSCCB dsccb;    // structure containing board settings
DSCB dscb;      // handle used to refer to the board
DSCQMM_MMR mmr; // master mode register, structure for programming the master mode
DSCQMM_CMR cmr;	// counter mode register, structure for programming the counter mode
WORD load_reg;  // load register
ERRPARAMS errorParams;  // structure for returning error code and error string
int intBuff;            // temp variables of size int

//we store the values here
unsigned short counts[QMM_MAX_COUNTER] = {0, 0, 0, 0,0,
								0, 0, 0, 0, 0};
//TODO, can we actually detect an overflow and update this correctly?	
bool overflows[QMM_MAX_COUNTER] = {false, false, false, false, false,
									false, false, false, false, false};


//for testing the resource manager read
char buffer[128] = "devct-qmm buffe\n";

//for the resource manager
static resmgr_connect_funcs_t    connect_funcs;
static resmgr_io_funcs_t         io_funcs;
static iofunc_attr_t             attr;

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
		"Usage: devct-qmm [options]\n"
		"\n"
		"		-h | --help						This help\n"
		"       -V | --version                  Display version\n"
		"\n"
	     << endl;

	exit(-1);
}

void
readAndResetCounter(int counter)
{
	if (counter < QMM_MIN_COUNTER || counter > QMM_MAX_COUNTER) {
		printf("readAndResetCounter: counter %d out of range, should be >= %d <= %d\n",
		counter, QMM_MIN_COUNTER,  QMM_MAX_COUNTER);
		return;
	}
	
	if (counter > LAST_USED_COUNTER) {
		printf("readAndResetCounter: counter %d > LAST_USED_COUNTER: %d\n",
				counter, LAST_USED_COUNTER);
		return;
	}
	//counter is valid, here we go
	
	printf( "\nQMM READ COUNTER: counter %d\n", counter );

	//read out the value into counts array	
	dscQMMSingleCounterControl(dscb, (BYTE) counter, QMM_ACTION_SAVE);
	dscQMMReadHoldRegister(dscb, (BYTE) counter, &counts[counter]);


	//TODO
	//error check here
	dscQMMSetLoadRegister(dscb, (BYTE)counter, INITIAL_VALUE);
	
	if( dscQMMSingleCounterControl(dscb, (BYTE)counter, QMM_ACTION_LOAD_AND_ARM) != DE_NONE)
	{
		dscGetLastError(&errorParams);
		fprintf(stderr, "readAndResetCounter: dscQMMSingleCounterControl: %s %s\n", ERROR_PREFIX, errorParams.errstring);
	}
}


/**
 * loads and arms counter number specified by counter
 * counter should be between 1 and 10
 * @returns true if ok, false otherwise
 */
bool
loadAndArmCounter(int counter)
{
	if (counter < QMM_MIN_COUNTER || counter > QMM_MAX_COUNTER) {
		printf("loadAndArmCounter: counter %d out of range, should be >= %d <= %d\n",
		counter, QMM_MIN_COUNTER,  QMM_MAX_COUNTER);
		return false;
	}
	
	if (counter > LAST_USED_COUNTER) {
		printf("loadAndArmCounter: counter %d > LAST_USED_COUNTER: %d\n",
				counter, LAST_USED_COUNTER);
		return false;
	}
	//counter is valid, here we go
	
	printf( "\nQMM COUNTER SETUP: counter %d\n", counter );

	cmr.counter = (BYTE) counter;
	cmr.count_direction = (BYTE) 1; //1 for counting up; 0 for down
	cmr.cycle = (BYTE) 0; //1 for cycle, 0 for 1 shot
	
	cmr.special_gate = 0;
	cmr.reload_source = 0;
	cmr.output_control = QMM_TOGGLE_ON_TC;
	cmr.gating_control = 0;
	cmr.count_type = 0;
	cmr.active_source_edge = 0;
	cmr.count_source = cmr.counter;
	
	if (cmr.count_source > 5) cmr.count_source -= 5;

	if( dscQMMSetCMR(dscb, &cmr) != DE_NONE)
	{
		dscGetLastError(&errorParams);
		fprintf(stderr, "loadAndArmCounter:dscQMMSetCMR %s %s\n", ERROR_PREFIX, errorParams.errstring);
		return false;
	}

	//TODO
	//error check here
	dscQMMSetLoadRegister(dscb, cmr.counter, INITIAL_VALUE);
	
	if( dscQMMSingleCounterControl(dscb, cmr.counter, QMM_ACTION_LOAD_AND_ARM) != DE_NONE)
	{
		dscGetLastError(&errorParams);
		fprintf(stderr, "loadAndArmCounter: dscQMMSingleCounterControl: %s %s\n", ERROR_PREFIX, errorParams.errstring);
		return 0;
	}


	//all seems good
	return true;
}

int
io_read (resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb)
{
    int         nleft;
    int         nbytes;
    int         nparts;
    int         status;


	printf("io_read\n");
	
    if ((status = iofunc_read_verify (ctp, msg, ocb, NULL)) != EOK)
        return (status);
        
    if (msg->i.xtype & _IO_XTYPE_MASK != _IO_XTYPE_NONE)
        return (ENOSYS);

    /*
     *  On all reads (first and subsequent), calculate
     *  how many bytes we can return to the client,
     *  based upon the number of bytes available (nleft)
     *  and the client's buffer size
     */

    nleft = ocb->attr->nbytes - ocb->offset;
    nbytes = min (msg->i.nbytes, nleft);

	printf("nleft: %d, nbytes: %d, ocb->attr->nbytes: %d, ocb->offset %d\n", nleft, nbytes, (int)ocb->attr->nbytes, (int)ocb->offset);

    if (nbytes > 0) {
        /* set up the return data IOV */
        SETIOV (ctp->iov, buffer + ocb->offset, nbytes);

        /* set up the number of bytes (returned by client's read()) */
        _IO_SET_READ_NBYTES (ctp, nbytes);

        /*
         * advance the offset by the number of bytes
         * returned to the client.
         */

        ocb->offset += nbytes;
        
        nparts = 1;
    } else {
        /*
         * they've asked for zero bytes or they've already previously
         * read everything
         */
        
        _IO_SET_READ_NBYTES (ctp, 0);
        
        nparts = 0;
    }

    /* mark the access time as invalid (we just accessed it) */

    if (msg->i.nbytes > 0)
        ocb->attr->flags |= IOFUNC_ATTR_ATIME;

    return (_RESMGR_NPARTS (nparts));
}

int 
main(int argc, char ** argv)
{
	getoptions( &argc, &argv,
			 "h|?|help&",           help,
			 "V|version&",          version,
			 0
		    );
		    
	//initialize the universal driver
   	if( dscInit( DSC_VERSION ) != DE_NONE )
	{
		dscGetLastError(&errorParams);
		fprintf( stderr, "main: dscInit: %s %s\n", ERROR_PREFIX, errorParams.errstring );
		return EXIT_FAILURE;
	}
	
	printf("Base address: %x\n", BASE_ADDRESS);
	printf("Interrupt level: %d\n", INT_LEVEL);
	
	dsccb.base_address = BASE_ADDRESS;
	dsccb.int_level = (BYTE)INT_LEVEL;

	//initialize counter timer board with the base address and int level	
	if(dscInitBoard(DSC_QMM, &dsccb, &dscb)!= DE_NONE)
	{
		dscGetLastError(&errorParams);
		printf("main: dscInitBoard: %s %s\n", ERROR_PREFIX, errorParams.errstring);
		return EXIT_FAILURE;
	}
	
	//TODO, experiment with this
	//Master mode register settings
	mmr.counter_group = 1;
	mmr.compare1_enable = 0;
	mmr.compare2_enable = 0;
	mmr.fout_divider = 8;
	mmr.fout_source = QMM_SOURCE_F3_40KHZ;
	mmr.tod_mode = 0;

	//set the Master mode register settings	
	if( dscQMMSetMMR(dscb, &mmr) != DE_NONE)
	{
		dscGetLastError(&errorParams);
		fprintf(stderr, "main: dscQMMSetMMR: %s %s\n", ERROR_PREFIX, errorParams.errstring);
		return EXIT_FAILURE;
	}

	
	//load and arm each counter
	for (int i = QMM_MIN_COUNTER; i <= LAST_USED_COUNTER ; i++) {
		if (!loadAndArmCounter(i)) {
			printf("Failed to loadAndArm Counter %d\n", i);
			return EXIT_FAILURE;
		}
	}
	
	//do the resource manager initialization
		 /* declare variables we'll be using */
    resmgr_attr_t        resmgr_attr;
    dispatch_t           *dpp;
    dispatch_context_t   *ctp;
    int                  id;

    // initialize dispatch interface
    if((dpp = dispatch_create()) == NULL) {
        fprintf(stderr, "main: %s: Unable to allocate dispatch handle.\n",
                argv[0]);
        return EXIT_FAILURE;
    }
	
  	// initialize resource manager attributes
    memset(&resmgr_attr, 0, sizeof resmgr_attr);
    resmgr_attr.nparts_max = 1;
    resmgr_attr.msg_max_size = 2048;

    // initialize functions for handling messages
    iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs, 
                     _RESMGR_IO_NFUNCS, &io_funcs);	
	
	io_funcs.read = io_read;                     

	//initialize attribute structure used by the device
    iofunc_attr_init(&attr, S_IFNAM | 0666, 0, 0);

    // attach our device name
    id = resmgr_attach(dpp,            // dispatch handle        
                       &resmgr_attr,   // resource manager attrs 
                       "/dev/devct-qmm",  // device name            
                       _FTYPE_ANY,     // open type              
                       0,              // flags                  
                       &connect_funcs, // connect routines       
                       &io_funcs,      // I/O routines           
                       &attr);         // handle                 
    if(id == -1) {
        fprintf(stderr, "main: %s: Unable to attach name.\n", argv[0]);
        return EXIT_FAILURE;
    }

    // allocate a context structure
    ctp = dispatch_context_alloc(dpp);

    // start the resource manager message loop */
    while(1) {
        if((ctp = dispatch_block(ctp)) == NULL) {
            fprintf(stderr, "block error\n");
            return EXIT_FAILURE;
        }
        dispatch_handler(ctp);
    }
	
}