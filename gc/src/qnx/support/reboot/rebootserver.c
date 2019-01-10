/**
 * This is a resource manager that opens up /dev/reboot
 * It then sits there and wait for writes to /dev/reboot
 * If it receives any writes to /dev/reboot and the uid of the local program that wrote to it
 * is vehicle uid (600), then it reboots the whole system
 * It does not work for remote programs trying to write to the /dev/reboot
 *
 * It should be run from /etc/rc.d/rc.local
 */

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
#include "v86_watchdog.h"

#define VEHICLE_UID 600
#define MAX_WD_TIMEOUT 120

static resmgr_connect_funcs_t    connect_funcs;
static resmgr_io_funcs_t         io_funcs;
static iofunc_attr_t             attr;

char  * dev_name = { "/dev/reboot" };

void disable_wd() {
	printf("rebootserver: disabling watchdogtimer\n");

#if ARCH == NOVA8890
  disable_nova8890_wd();
#elif ARCH == AMPRO400
  disable_ampro400_wd();
#else
#error "Must define ARCH"
#endif
}

void enable_wd(unsigned int time)
{
	printf("rebootserver: enabling watchdogtimer: %d seconds\n", time);
#if ARCH == NOVA8890
  enable_nova8890_wd(time);
#elif ARCH == AMPRO400
  enable_ampro400_wd(time);
#else
#error "Must define ARCH"
#endif

}

//Gets the uid of the process of pid
//only works locally
//adapted from watchdog/getprocinfo.cpp
int get_uid (pid_t pid)
{
	int stat;
	char path[512];
	int fd;
	procfs_info pidinfo;
	
	snprintf(path, sizeof(path), "/proc/%d", pid);
	fd = open (path, O_RDONLY);
	
	if (fd < 0) {
		printf("reboot_server: get_uid: Failed to open %s\n", path);
		return -1;
	}	
	
	//get status info about process
	stat = devctl(fd, DCMD_PROC_INFO, &pidinfo, sizeof (pidinfo), 0);
	
	if (stat != EOK) {
		perror("devctl");
		printf("reboot_server: get_uid: Failed to devctl\n");
	}
	
	//got what we want
	return pidinfo.uid;
}

/*
void do_reboot()
{
	printf("Causing reboot...\n");	
	if (sysmgr_reboot()) {
		perror("sysmgr_reboot\n");
	
	}
}
*/

//
// if we receive a ascii "0" then we disable the watchdog
// if we receive any other number that is within some bounds, then we enable the watchdog
// with that number as number of seconds for timeout
//

int                                                                               
io_write (resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb)              
{                                                                                 
    int     status;                                                               
    char    *buf;                                                                 
    struct _msg_info info;
    int uid;
    int val; 
                                                                              
    if ((status = iofunc_write_verify(ctp, msg, ocb, NULL)) != EOK)               
        return (status);                                                          
                                                                                  
    if (msg->i.xtype & _IO_XTYPE_MASK != _IO_XTYPE_NONE)                          
        return(ENOSYS);                                                           
                                                                                  
    /* set up the number of bytes (returned by client's write()) */
                                                                                  
    _IO_SET_WRITE_NBYTES (ctp, msg->i.nbytes);                                    
                                                                                  
    buf = (char *) malloc(msg->i.nbytes + 1);                                     
    if (buf == NULL)                                                              
        return(ENOMEM);                                                           
                                                                                  
    /*                                                                            
     *  reread the data from the sender's message buffer.  
     *  We're not assuming that all of the data fit into the 
     *  resource manager library's receive buffer.                                         
     */                                                                            
                                                                                  
    resmgr_msgread(ctp, buf, msg->i.nbytes, sizeof(msg->i));                      
    buf [msg->i.nbytes] = '\0'; /* just in case the text is not NULL terminated */

    //printf ("Received %d bytes = '%s'\n", msg -> i.nbytes, buf);                  
    //printf("ctp->rcvid: %d\n", ctp->rcvid);

    //find out about info about rcvid
    MsgInfo(ctp->rcvid, &info);
    
    //printf("pid of sending process: %d\n", info.pid);
    //printf("uid of sending porcess: %d\n", get_uid(info.pid));

    val = atoi(buf);
    uid = get_uid (info.pid);
    
    if (uid == VEHICLE_UID) {
      if (val == 0 && buf[0] == '0') {
	disable_wd();
      }
      else if (val >0 && val <= MAX_WD_TIMEOUT) {
	enable_wd(val);
      }
      else {
	printf("Value given: %d not appropriate for watchdog timer, should be >= 0 and <= %d\n", val, MAX_WD_TIMEOUT);
      }
    }
    else {
      printf("uid: %d of process not vehicle uid: %d\n", uid, VEHICLE_UID);
    }
                                                                                
    if (msg->i.nbytes > 0)                                                        
        ocb->attr->flags |= IOFUNC_ATTR_MTIME | IOFUNC_ATTR_CTIME;                

    free(buf);  
    return (_RESMGR_NPARTS (0));                                                  

}
int main(int argc, char **argv)
{
	
	 /* declare variables we'll be using */
    resmgr_attr_t        resmgr_attr;
    dispatch_t           *dpp;
    dispatch_context_t   *ctp;
    int                  id;

    /* initialize dispatch interface */
    if((dpp = dispatch_create()) == NULL) {
        fprintf(stderr, "%s: Unable to allocate dispatch handle.\n",
                argv[0]);
        return EXIT_FAILURE;
    }
	
	  /* initialize resource manager attributes */
    memset(&resmgr_attr, 0, sizeof resmgr_attr);
    resmgr_attr.nparts_max = 1;
    resmgr_attr.msg_max_size = 2048;

    /* initialize functions for handling messages */
    iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs, 
                     _RESMGR_IO_NFUNCS, &io_funcs);	
	
	io_funcs.write = io_write;                     

	  /* initialize attribute structure used by the device */
    iofunc_attr_init(&attr, S_IFNAM | 0666, 0, 0);

    /* attach our device name */
    id = resmgr_attach(dpp,            /* dispatch handle        */
                       &resmgr_attr,   /* resource manager attrs */
                       "/dev/reboot",  /* device name            */
                       _FTYPE_ANY,     /* open type              */
                       0,              /* flags                  */
                       &connect_funcs, /* connect routines       */
                       &io_funcs,      /* I/O routines           */
                       &attr);         /* handle                 */
    if(id == -1) {
        fprintf(stderr, "%s: Unable to attach name.\n", argv[0]);
        return EXIT_FAILURE;
    }

    /* allocate a context structure */
    ctp = dispatch_context_alloc(dpp);

    /* start the resource manager message loop */
    while(1) {
        if((ctp = dispatch_block(ctp)) == NULL) {
            fprintf(stderr, "block error\n");
            return EXIT_FAILURE;
        }
        dispatch_handler(ctp);
    }
                
}

                              
