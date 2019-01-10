////////////////////////////////////////////////////////////////////////////
//
//    File: ask.cc
//
//    Usage:
//        #include "ask.h"
//        #include "ideconsole.h"
//
//        IDEConsole::StdoutNoBuffering();
//
//    Description:
//        A uniform interface for asking questions.  Could probably 
//        use templates for int/hex and float/double.
//
//        Need to include ideconsole.h and call
//        IDEConsole::StdoutNoBuffering(); at the beginning of each program
//        so this code can be run under Momentics in QNX 6.2.1.
//        Should be able to remove in 6.3?
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "ask.h"
#include <exception>
#include <stdexcept>

// private macros
#define INPUTLEN (82)

// private functions
static char stdinCharGet();
static void stdinStrGet(char *str, const int strLen);
static void stdinFlush();

//
//    Ask::Int - prompt the user for an integer
//
//    Default will be selected if the user types a CR
//
//    Returns integer obtained from user
//
int Ask::Int(const char *prompt, const int def)
{
    char input[INPUTLEN];
    int result;
    float resultFloat;
    bool resultOK = false;
    do {
		if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
        // prompt user for input
        printf("%s [%i] ", prompt, def);

        // get first character to see if selected default
        fflush(stdout);													// insure output flushed
        char c = getc(stdin);
        if ( c == '\n') {
            // selected default answer
            result   = def;
            resultOK = true;
        } else {
            // put first character back
            ungetc(c, stdin);

            // get input and flush remaining characters
            stdinStrGet(input, sizeof(input));

            // check input
            resultOK = (sscanf(input, "%d", &result) > 0);
            if ( resultOK ) {
                // if entered decimal point number, truncate and warn
				sscanf(input, "%f", &resultFloat);
				if ( resultFloat != result ) {
					printf("Warning: integer expected, value truncated.\n");
				}
            } else {
                // didn't enter proper number, prompt again
                printf("\007Need to enter an integer.\n");
	        }
        }
    } while ( !resultOK );

    return result;
}

//
//    Ask::Int - prompt the user for a bounded integer
//
//    Default will be selected if the user types a CR
//    Minimum and maximum values supplied in this version
//    Will not accept default if out of range
//    Might want to show range in prompt
//
//    Returns integer obtained from user
//
int Ask::Int(const char *prompt, const int def, 
             const int min, const int max)
{
    int result;

    if ( min > max ) {
        // deal with case where min > max
        result = Ask::Int(prompt, def);
    } else {
        // prompt until get result in range
        do {
			if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
            result = Ask::Int(prompt, def);

            // check if number is in range
            if ( (result < min) || (result > max) ) {
                printf("\007Must be in range: (%d, %d)\n", min, max);
    	    }
        } while ( (result < min) || (result > max) );
    }

    return result;
}

//
//    Ask::Hex - prompt the user for a hexidecimal number
//
//    Default will be selected if the user types a CR
//
//    Returns hexidecimal number obtained from user
//
int Ask::Hex(const char *prompt, const int def)
{
    char c;
    char input[INPUTLEN];
    int result;
    bool resultOK = false;

    do {
 		if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
       // prompt user for input
        printf("%s [%x] ", prompt, def);

        // get first character to see if selected default
        fflush(stdout);													// insure output flushed
        c = getc(stdin);
        if ( c == '\n' ) {
            // selected default answer
            result   = def;
            resultOK = true;
        } else {
            // put first character back
            ungetc(c, stdin);

            // get input and flush remaining characters
            stdinStrGet(input, sizeof(input));

            // check input
            resultOK = (sscanf(input, "%x", &result) > 0);
            if ( !resultOK ) {
                // didn't enter proper number, prompt again
                printf("\007Need to enter a hexidecimal number.\n");
	        }
        }
    } while ( !resultOK );

    return result;
}

//
//    Ask::Hex - prompt the user for a bounded hexidecimal number
//
//    Default will be selected if the user types a CR
//    Minimum and maximum values supplied in this version
//    Will not accept default if out of range
//    Might want to show range in prompt
//
//    Returns hexidecimal number obtained from user
//
int Ask::Hex(const char *prompt, const int def, 
             const int min, const int max)
{
    int result;

    if ( min > max ) {
        // deal with case where min > max
        result = Ask::Hex(prompt, def);
    } else {
        // prompt until get result in range
        do {
			if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
            result = Ask::Hex(prompt, def);

            // check if number is in range
            if ( (result < min) || (result > max) ) {
                printf("\007Must be in range: (%x, %x)\n", min, max);
	    }
        } while ( (result < min) || (result > max) );
    }

    return result;
}

//
//    Ask::Float - prompt the user for a floating point number
//
//    Default will be selected if the user types a CR
//
//    Returns float obtained from user
//
float Ask::Float(const char *prompt, const float def)
{
    char c;
    char input[INPUTLEN];
    float result;
    bool resultOK = false;

    do {
		if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
        // prompt user for input
        printf("%s [%g] ", prompt, def);

        // get first character to see if selected default
        fflush(stdout);													// insure output flushed
        c = getc(stdin);
        if ( c == '\n' ) {
            // selected default answer
            result   = def;
            resultOK = true;
        } else {
            // put first character back
            ungetc(c, stdin);

            // get input and flush remaining characters
            stdinStrGet(input, sizeof(input));

            // check format of input
            resultOK = (sscanf(input, "%f", &result) > 0);
            if ( !resultOK ) {
                // didn't enter proper number, prompt again
                printf("\007Need to enter a floating point number.\n");
			}
        }
    } while ( !resultOK );

    return result;
}

//
//    Ask::Float - prompt the user for a bounded floating point number
//
//    Default will be selected if the user types a CR
//    Minimum and maximum values supplied in this version
//    Will not accept default if out of range
//    Might want to show range in prompt
//
//    Returns float obtained from user
//
float Ask::Float(const char *prompt, const float def, 
                 const float min, const float max)
{
    float result;

    if ( min > max ) {
        // deal with case where min > max
        result = Ask::Float(prompt, def);
    } else {
        // prompt until get result in range
        do {
			if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
            result = Ask::Float(prompt, def);

            // check if number is in range
            if ( (result < min) || (result > max) ) {
                printf("\007Must be in range: (%g, %g)\n", min, max);
			}
        } while( (result < min) || (result > max) );
    }

    return result;
}

//
//    Ask::Double - prompt the user for a double floating point number
//
//    Default will be selected if the user types a CR
//
//    Returns double obtained from user
//
double Ask::Double(const char *prompt, const double def)
{
    char c;
    char input[INPUTLEN];
    double result;
    bool resultOK = false;

    do {
		if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
        // prompt user for input
        printf("%s [%g] ", prompt, def);

        // get first character to see if selected default
        fflush(stdout);													// insure output flushed
        c = getc(stdin);
        if ( c == '\n' ) {
            // selected default answer
            result   = def;
            resultOK = true;
        } else {
            // put first character back
            ungetc(c, stdin);

            // get input and flush remaining characters
            stdinStrGet(input, sizeof(input));

            // check format of input
            resultOK = (sscanf(input, "%lf", &result) > 0);
            if ( !resultOK ) {
                // didn't enter number, prompt again
                printf("\007Need to enter a floating point number.\n");
	        }
        }
    } while ( !resultOK );

    return result;
}

//
//    Ask::Double - prompt the user for a bounded double floating point number
//
//    Default will be selected if the user types a CR
//    Minimum and maximum values supplied in this version
//    Will not accept default if out of range
//    Might want to show range in prompt
//
//    Returns double obtained from user
//
double Ask::Double(const char *prompt, const double def, 
                   const double min, const double max)
{
    double result;

    if ( min > max ) {
        // deal with case where min > max
        result = Ask::Double(prompt, def);
    } else {
        // prompt until get result in range
        do {
			if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
            result = Ask::Double(prompt, def);

            // check if number is in range
            if ( (result < min) || (result > max) ) {
                printf("\007Must be in range: (%g, %g)\n", min, max);
	        }
        } while ( (result < min) || (result > max) );
    }

    return result;
}

//
//    Ask::Char - prompt the user for a single character
//
//    Default will be selected if the user types a CR
//
//    Returns character obtained from user
//
char Ask::Char(const char *prompt, const char def)
{
    char c;
	if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF

	// prompt user for input
	printf("%s [%c] ", prompt, def);

	// get first character and flush remaining characters
	c = stdinCharGet();

	if ( c == '\n' ) {
		// selected default answer
		c = def;
	}

    return c;
}

//
//    Ask::Char - prompt the user for a single character from char string
//
//    Default will be selected if the user types a CR
//    Legal characters supplied in this version
//    Will not accept default if not in char string
//    Might want to show legal char string in prompt
//
//    Returns character obtained from user
//
char Ask::Char(const char *prompt, const char def, const char *charString)
{
    char result;
    bool resultOK;

	// prompt until get result in char string
	do {
		if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
		result = Ask::Char(prompt, def);
		resultOK = (strchr(charString, result) != NULL);

		// check if char is in char string
		if ( !resultOK ) {
			printf("\007Need to type one of these characters: (%s)\n", 
			       charString);
		}
	} while ( !resultOK );

    return result;
}

//
//    Ask::String - prompt the user for a fixed length string
//
//    Default will be selected if the user types a CR
//    Default will be truncated if longer than resultLen
//    Will only get resultLen characters from stdin
//
//    Returns nothing (string from user in result)
//
void Ask::String(const char *prompt, const char *def, 
                 char *result, size_t resultLen)
{
    char c;

	// prompt user for input
	printf("%s [%s] ", prompt, def);

	// get first character
    fflush(stdout);													// insure output flushed
	c = getc(stdin);
	if ( c == '\n' ) {
		// selected default answer
		strncpy(result, def, resultLen-1); 

		// terminate result with null character just in case 
		result[resultLen-1] = '\0';
	} else {
		// put first character back
		ungetc(c, stdin);

		// get input and flush remaining characters
		stdinStrGet(result, resultLen);
	}
}

//
//    Ask::YesNo - prompts the user for yes/no
//
//    Default will be selected if the user types a CR
//    Set default true = yes, or false = no
//
//    Returns true = yes, or false = no
//
bool Ask::YesNo(const char *prompt, const bool def)
{
    char c;
    bool result = false;
    bool resultOK = false;

    do {
		if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
		// prompt user for input
		printf("%s [%s] ", prompt, def ? "(y)/n" : "y/(n)");

        // get first character and flush remaining characters
		c = stdinCharGet();

		switch (c) {
			case '\n': // selected default
			    result   = def;
			    resultOK = true;
			    break;
		    case 'y': // selected yes
		    case 'Y':
		        result   = true;
		        resultOK = true;
			    break;
			case 'n': // selected no
			case 'N':
			    result   = false; 
		        resultOK = true;
			    break;
			default: // illegal input
			    printf("\007Need to type 'y' or 'n'\n");
			    break;
		}
	} while ( !resultOK );

    return result;
}

//
//    Ask::Pause - pauses until user presses a key
//
//    If prompt is set to NULL, then default prompt is used.
//
//    Returns nothing
//
void Ask::Pause(const char *prompt)
{ 
	if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
	// prompt user to continue
	if ( prompt == NULL )  {
	    printf("Type 'Enter' to continue ...");
	} else {
		printf(prompt);
		printf("...");
	}

	// wait for input and flush characters
	stdinFlush();
}

void Ask::Pause()
{
    Ask::Pause(NULL);
}

//
//    Ask::CharReady - checks if there are any characters in stdin
//
//    Haven't been able to get this to work in an IDE Console view.
//
//    Returns number of characters in stdin queue
//
//int Ask::CharReadyOld()
//{
//   int n;
//
//	if ( ioctl(0, FIONREAD, &n) == -1 ) {
//	    return -1;
//	} else {
//	    return n;
//	}
//}

bool Ask::CharReady()
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    tv.tv_sec = 0;
    tv.tv_usec = 1000;

    if ( select(STDIN_FILENO+1, &fds, 0, 0, &tv) > 0 ) {
        return true;
    } else {
        return false;
    }
}

//
//    Ask::CharFlush - flushes any characters in stdin
//
//    Returns 0 if successful, -1 if an error occurred
//
void Ask::CharFlush()
{
    tcflush(0, TCIFLUSH);
}


//
//    stdinCharGet - gets a character from stdin
//                   then clears out the rest of stdin, if necessary
//
//    Returns the first character in stdin
//
static char stdinCharGet() {
    char c;
	fflush(stdout);											// ***TEMP TEST***
	scanf("%c", &c);

	if ( c == '\n' ) {
	    // user entered CR, return right away
	    return c;
	}

    // user didn't enter CR, be sure and get rest of chars out of buffer
    for (;;)
    {	if (getchar() == '\n') break;
    	if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
	}

	return c;
}

//
//    stdinStrGet - gets a string of a specified size from stdin
//                  then clears out the rest of stdin, if necessary
//
//    Returns nothing
//
static void stdinStrGet(char *str, const int strLen) {
    char c;
    int i;
    fflush(stdout);													// insure output flushed
    i = 0;
    while ( ((c = getchar()) != '\n') && (i < (strLen-1)) ) {
        str[i++] = c;
		if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
	}
	str[i] = '\0';
	if ( (c != '\n') && (i == (strLen-1)) ) {
	    stdinFlush();
	}
}

//
//    stdinFlush - clear out stdin up to CR
//
//    Expects a CR in stdin buffer
//
//    Returns nothing
//
static void stdinFlush() {
	fflush(stdout);													// insure output flushed
    for (;;)
    {	if (getchar() == '\n') break;
    	if (feof(stdin)) throw(std::runtime_error("EOF"));		// called when at EOF
	}
}