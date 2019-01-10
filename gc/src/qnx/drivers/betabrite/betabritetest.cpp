//
//	betabrite.cpp  -- BetaBrite interface class
//
//	J. Nagle
//	Team Overbot
//	November, 2003
//	
//	Based on the project "betabrite" from SourceForge.
//	Original copyright notice appears below.	
//
/*
   betabrite.c -- command line utility to display a message on a BetaBrite LED sign
 
   Copyright (C) 2002 boB Rudis, bob@rudis.net
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307,
   USA
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "betabrite.h"
//
//	Temporary test driver
//
static void DisplayUsage() {

    printf("Usage: betabrite [-s speed] [-b memfile ] [-h] [-v]\n");
    printf("                 [-m text|-f filename] [-p port]\n\n");
    printf("Options:\n\n");
    printf(" -s speed	: how long to hold text; 0=no delay, 1-5=longest to shortest\n ");
    printf("          	  (default=1)\n") ;
    printf(" -c color	:  Color (e.g. \"R\")\n");
    printf("          	  (default=A (amber) )\n") ;
    printf(" -h		: this help screen\n");
    printf(" -v		: display input and status - if any\n") ;
    printf(" -m text	: text of message to display; (default - blank [""])\n");
    printf(" -p port	: serial port to use (e.g. \"/dev/ser1\")\n");
    printf("          	  (default=\"/dev/ser1\" )\n\n") ;
 
} /* DisplayUsage() */
//
/* main() - parse options, display message */

int main(int argc, char *argv[]) {

    int c ;
    extern char *optarg ;

    char *betafile = NULL ;
    char *defaultbetafile = "A" ;

    char *msg = "" ;

    char *msgfilename = NULL ;

    char *serial_port = "/dev/ser1" ;
    char* color = BETABRITE_COLOR_AMBER;		// default color

    int speedsel = 1 ; /* 0 = none, 1-5 = slowest to fastest */
    char *holdspeed = BETABRITE_SPEED_SLOWEST ;

    int verbose_flag = 0 ;
    int msgtext_flag = 0 ;
    int msgfile_flag = 0 ;
    int port_flag    = 0 ;
    int options      = 0 ;

    int errflag = 0 ;

    while ((c = getopt(argc,argv,"c:p:s:b:m:f:hv")) != -1) {

        switch (c) {
        
        case 'c'	:														// color
            switch (optarg[0]) {

            case 'R' : color = BETABRITE_COLOR_RED ; break ;
            case 'G' : color = BETABRITE_COLOR_GREEN ; break ;
            case 'Y' : color = BETABRITE_COLOR_YELLOW ; break ;
            case 'A' : color = BETABRITE_COLOR_AMBER ;     break ;
 	        default: errflag++ ; break ;

            } /* switch */

            options++ ; break ;
            
        case 's' : speedsel = atoi(optarg) ;

            switch (speedsel) {

            case 0 : holdspeed = BETABRITE_NO_HOLD_SPEED ; break ;
            case 1 : holdspeed = BETABRITE_SPEED_SLOWEST ; break ;
            case 2 : holdspeed = BETABRITE_SPEED_LOW ;     break ;
            case 3 : holdspeed = BETABRITE_SPEED_MEDIUM ;  break ;
            case 4 : holdspeed = BETABRITE_SPEED_HIGH ;    break ;
            case 5 : holdspeed = BETABRITE_SPEED_FASTEST ; break ;

            default: errflag++ ; break ;

            } /* switch */

            options++ ; break ;

        case 'p' : serial_port = optarg ; port_flag++ ;
            options++ ; break ;

        case 'b' : betafile = optarg ;
        if (strlen(betafile) > 1) { errflag++ ; } /* if */
            options++ ; break ;

        case 'm' : msg = optarg ; msgtext_flag++ ;
            options++ ; break ;

        case 'f' : msgfilename = optarg ; msgfile_flag++ ;
            options++ ;break ;

        case 'h' : errflag++ ;
            options++ ; break ;

        case 'v' : verbose_flag++ ;
            break ;

        case '?' : errflag++ ;
            break ;

        default  : errflag++ ;
            break ;

        } /* switch */

    } /* while */

if (!options) { errflag++ ; } /* if */

    /* cannot have a msg cmd line and a msg file */

    if ((msgtext_flag > 0) && (msgfile_flag > 0)) { errflag++ ; } /* if */

    if (msg == NULL) { errflag++ ; } /* if */


    /* if we need help or if there's a cmd line error or a processing error... */

    if (errflag) {
        DisplayUsage() ;
        exit(2) ;
    } /* if */


    /* if no betabrite memory file is given, use the default */

    if (betafile == NULL) { betafile = defaultbetafile ; } /* if */


    /* say what we're doing if asked */

    if (verbose_flag) {
        printf("port: [%s]\nmsg: [%s]\nbetafile: [%s]\nspeed: [%d]\n",serial_port,msg,betafile,speedsel) ;
        if (msgfilename) {
            printf("msg file: [%s]\n",msgfilename) ;
        } /* if */
    } /* if */


    /* Setup serial communications */
    BetaBrite sign;
    int stat = sign.open(serial_port);											// open the sign
    if (stat) 
    {	perror("Unable to initialize serial port for BetaBrite");
    	exit(2);
    }
    ////sign.setholdspeed(BETABRITE_SPEED_MEDIUM);
   	sign.setcolor(color);
	stat = sign.display(msg);
    if (stat) 
    {	perror("Unable to send message to BetaBrite");
    	exit(2);
    }
	return(EOK);

}