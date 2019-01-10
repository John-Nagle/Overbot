/**
 * Khian Hao Lim, Team Overbot
 * 11/24/03
 *
 * logic that writes to the hardware watchdogs on the ampro 400 and nova8890
 * single board computers
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/neutrino.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <inttypes.h>
#include <x86/v86.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sys/dispatch.h>
#include <sys/iofunc.h>
#include <sys/queue.h>

#define MAXTIMEOUT 120

//
// from http://www.jenlogix.co.nz/images/manuals/nova-8890.pdf
// 

void 
set_nova8890_wd(unsigned int timeout)
{
  struct _v86reg reg;
 
  if (timeout > MAXTIMEOUT) {
    printf("timeout > %d, try using a smaller timeout\n", MAXTIMEOUT);
    return;
  }

  reg.eax = 0x6F02;
  reg.ebx = timeout;
  if(_intr_v86( 0x15, &reg, 0, 0 )==-1) {
 	perror("nova8890: _intr_v86"); 

  }  
}

void
disable_nova8890_wd()
{
  set_nova8890_wd(0x0);
}

void
enable_nova8890_wd(unsigned int time)
{
  set_nova8890_wd(time);
}


//
// from http://www.ampro.com/assets/applets/CM400RefMan.pdf
//

void
disable_ampro400_wd()
{
  struct _v86reg reg;
  reg.eax = (0xC3 << 8);
  reg.ebx = 0x00;
  if(_intr_v86( 0x15, &reg, 0, 0 )==-1) {
    perror("ampro400: _intr_v86");
  }
}

void
enable_ampro400_wd(unsigned int time)
{
  struct _v86reg reg;
  reg.eax = (0xC3 << 8) | 1;
  reg.ebx = time;
  if(_intr_v86( 0x15, &reg, 0, 0 )==-1) {
    perror("ampro400: _intr_v86");
  }
}
