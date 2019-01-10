/////////////////////////////////////////////////////////////////////////////
//
//    File: check_ask.cc
//
//    Usage:
//        check_ask
//
//    Description:
//        A test file for ask.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July, 2003
//
////////////////////////////////////////////////////////////////////////////

#include "ask.h"
#include "ideconsole.h"

#define STRLEN (82)

void test_int()
{
    int resp=42, min=-123, minBig=456, max=123;
    char prompt[STRLEN];

    printf("\n");
    printf("testing Ask::Int ...\n");
    printf("\n");

    // test default
    resp = Ask::Int("Enter a CR to see default selected", resp);
    printf("you entered ... %i\n", resp);
    printf("\n");

    // test enter positive integer
    resp = Ask::Int("Enter a positive integer", resp);
    printf("you entered ... %i\n", resp);
    printf("\n");

    // test enter negative integer
    resp = Ask::Int("Enter a negative integer", resp);
    printf("you entered ... %i\n", resp);
    printf("\n");

    // test enter float
    resp = Ask::Int("Enter a floating point number", resp);
    printf("you entered ... %i\n", resp);
    printf("\n");

    // test enter characters
    resp = Ask::Int("Enter something that isn't a number", resp);
    printf("you entered ... %i\n", resp);
    printf("\n");

    // test enter long string
    resp = Ask::Int("Enter a very long string", resp);
    printf("you entered ... %i\n", resp);
    printf("\n");

    // test enter number in range
    sprintf(prompt, "Enter a number in range (%i,%i)", min, max);
    resp = Ask::Int(prompt, resp, min, max);
    printf("you entered ... %i\n", resp);
    printf("\n");

    // test enter number out of range
    sprintf(prompt, "Enter a number out of range (%i,%i)", min, max);
    resp = Ask::Int(prompt, resp, min, max);
    printf("you entered ... %i\n", resp);
    printf("\n");

    // test min > max
    sprintf(prompt, "Enter a number where min>max, accepts any # (%i,%i)", 
            minBig, max);
    resp = Ask::Int(prompt, resp, minBig, max);
    printf("you entered ... %i\n", resp);
    printf("\n");

    // test default outside range
    sprintf(prompt, "Enter a number where default=def+max (%i,%i)", 
            min, max);
    resp = Ask::Int(prompt, resp+max, min, max);
    printf("you entered ... %i\n", resp);
    printf("\n");
}

void test_hex()
{
    int resp=10, min=-123, minBig=456, max=123;
    char prompt[STRLEN];

    printf("\n");
    printf("testing Ask::Hex ...\n");
    printf("\n");

    // test default
    resp = Ask::Hex("Enter a CR to see default selected", resp);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");

    // test enter positive hex
    resp = Ask::Hex("Enter a positive hex", resp);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");

    // test enter negative integer
    resp = Ask::Hex("Enter a negative hex", resp);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");

    // test enter 0xNNN integer
    resp = Ask::Hex("Enter a hex number with 0x preceeding it", resp);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");

    // test enter float
    resp = Ask::Hex("Enter a floating point number", resp);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");

    // test enter characters
    resp = Ask::Hex("Enter something that isn't a number", resp);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");

    // test enter long string
    resp = Ask::Hex("Enter a very long string", resp);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");

    // test enter number in range
    sprintf(prompt, "Enter a number in range (%x,%x)", min, max);
    resp = Ask::Hex(prompt, resp, min, max);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");

    // test enter number out of range
    sprintf(prompt, "Enter a number out of range (%x,%x)", min, max);
    resp = Ask::Hex(prompt, resp, min, max);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");

    // test min > max
    sprintf(prompt, "Enter a number where min>max, accepts any # (%x,%x)", 
            minBig, max);
    resp = Ask::Hex(prompt, resp, minBig, max);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");

    // test default outside range
    sprintf(prompt, "Enter a number where default=def+max (%x,%x)", 
            min, max);
    resp = Ask::Hex(prompt, resp+max, min, max);
    printf("you entered ... 0x%x (%i)\n", resp, resp);
    printf("\n");
}

void test_float()
{
    float resp=42.0, min=-123.0, minBig=456.0, max=123.0;
    char prompt[STRLEN];

    printf("\n");
    printf("testing Ask::Float ...\n");
    printf("\n");

    // test default
    resp = Ask::Float("Enter a CR to see default selected", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter positive floating point number
    resp = Ask::Float("Enter a positive floating point number", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter negative floating point number
    resp = Ask::Float("Enter a negative floating point number", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter an integer
    resp = Ask::Float("Enter an integer", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter characters
    resp = Ask::Float("Enter something that isn't a number", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter long string
    resp = Ask::Float("Enter a very long string", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter number in range
    sprintf(prompt, "Enter a number in range (%f,%f)", min, max);
    resp = Ask::Float(prompt, resp, min, max);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter number out of range
    sprintf(prompt, "Enter a number out of range (%f,%f)", min, max);
    resp = Ask::Float(prompt, resp, min, max);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test min > max
    sprintf(prompt, "Enter a number where min>max, accepts any # (%f,%f)", 
            minBig, max);
    resp = Ask::Float(prompt, resp, minBig, max);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test default outside range
    sprintf(prompt, "Enter a number where default=def+max (%f,%f)", 
            min, max);
    resp = Ask::Float(prompt, resp+max, min, max);
    printf("you entered ... %f\n", resp);
    printf("\n");
}

void test_double()
{
    double resp=42.0, min=-123.0, minBig=456.0, max=123.0;
    char prompt[STRLEN];

    printf("\n");
    printf("testing Ask::Double ...\n");
    printf("\n");

    // test default
    resp = Ask::Double("Enter a CR to see default selected", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter positive floating point number
    resp = Ask::Double("Enter a positive floating point number", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter negative floating point number
    resp = Ask::Double("Enter a negative floating point number", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter an integer
    resp = Ask::Double("Enter an integer", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter characters
    resp = Ask::Double("Enter something that isn't a number", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter long string
    resp = Ask::Double("Enter a very long string", resp);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter number in range
    sprintf(prompt, "Enter a number in range (%f,%f)", min, max);
    resp = Ask::Double(prompt, resp, min, max);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test enter number out of range
    sprintf(prompt, "Enter a number out of range (%f,%f)", min, max);
    resp = Ask::Double(prompt, resp, min, max);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test min > max
    sprintf(prompt, "Enter a number where min>max, accepts any # (%f,%f)", 
            minBig, max);
    resp = Ask::Double(prompt, resp, minBig, max);
    printf("you entered ... %f\n", resp);
    printf("\n");

    // test default outside range
    sprintf(prompt, "Enter a number where default=def+max (%f,%f)", 
            min, max);
    resp = Ask::Double(prompt, resp+max, min, max);
    printf("you entered ... %f\n", resp);
    printf("\n");
}

void test_char()
{
    char resp = 'g';
    char testStr[] = "acegik";
    char prompt[STRLEN];

    printf("\n");
    printf("testing Ask::Char ...\n");
    printf("\n");

    // test default
    resp = Ask::Char("Enter a CR to see default selected", resp);
    printf("you entered ... %c\n", resp);
    printf("\n");

    // test enter character
    resp = Ask::Char("Enter a character", resp);
    printf("you entered ... %c\n", resp);
    printf("\n");

    // test enter string
    resp = Ask::Char("Enter a string", resp);
    printf("you entered ... %c\n", resp);
    printf("\n");

    // test enter number
    resp = Ask::Char("Enter a number", resp);
    printf("you entered ... %c\n", resp);
    printf("\n");

    // test enter long string
    resp = Ask::Char("Enter a very long string", resp);
    printf("you entered ... %c\n", resp);
    printf("\n");

    // test enter character in range
    sprintf(prompt, "Enter a character in string (%s)", testStr);
    resp = Ask::Char(prompt, resp, testStr);
    printf("you entered ... %c\n", resp);
    printf("\n");

    // test enter character out of range
    sprintf(prompt, "Enter a character not in string (%s)", testStr);
    resp = Ask::Char(prompt, resp, testStr);
    printf("you entered ... %c\n", resp);
    printf("\n");

    // test default outside range
    sprintf(prompt, "Enter a number where default not in string (%s)", 
            testStr);
    resp = Ask::Char(prompt, 'z', testStr);
    printf("you entered ... %c\n", resp);
    printf("\n");
}

void test_string()
{
    char resp[STRLEN] = "acegik";

    printf("\n");
    printf("testing Ask::String ...\n");
    printf("\n");

    // test default
    Ask::String("Enter a CR to see default selected", resp, resp, STRLEN);
    printf("you entered ... %s\n", resp);
    printf("\n");

    // test enter character
    Ask::String("Enter a character", resp, resp, STRLEN);
    printf("you entered ... %s\n", resp);
    printf("\n");

    // test enter string
    Ask::String("Enter a string", resp, resp, STRLEN);
    printf("you entered ... %s\n", resp);
    printf("\n");

    // test enter number
    Ask::String("Enter a number", resp, resp, STRLEN);
    printf("you entered ... %s\n", resp);
    printf("\n");

    // test enter long string
    Ask::String("Enter a very long string", resp, resp, STRLEN);
    printf("you entered ... %s\n", resp);
    printf("\n");
}

void test_yesno()
{
    bool resp = true;

    printf("\n");
    printf("testing Ask::YesNo ...\n");
    printf("\n");

    // test default
    resp = Ask::YesNo("Enter a CR to see default selected", resp);
    printf("you entered ... %c\n", resp ? 'y' : 'n');
    printf("\n");

    // test enter character
    resp = Ask::YesNo("Enter a character", resp);
    printf("you entered ... %c\n", resp ? 'y' : 'n');
    printf("\n");

    // test enter string
    resp = Ask::YesNo("Enter a string", resp);
    printf("you entered ... %c\n", resp ? 'y' : 'n');
    printf("\n");

    // test enter number
    resp = Ask::YesNo("Enter a number", resp);
    printf("you entered ... %c\n", resp ? 'y' : 'n');
    printf("\n");

    // test enter long string
    resp = Ask::YesNo("Enter a very long string", resp);
    printf("you entered ... %c\n", resp ? 'y' : 'n');
    printf("\n");
}

void test_pause()
{
    printf("\n");
    printf("testing Ask::Pause ...\n");
    printf("\n");

    // test prompt == NULL
    Ask::Pause(NULL);
    printf("continuing ...\n");
    printf("\n");

    // test prompt != NULL
    Ask::Pause("Enter any key and you shall continue ...");
    printf("continuing ...\n");
    printf("\n");
}

void test_charReadyFlush()
{
    while ( !Ask::CharReady() ) {
        printf("type any char to stop this printing\n");
        sleep(1);
    }
    printf(Ask::CharReady() ? "characters ready\n" : "no characters ready\n");
    printf("flushing queue...\n");
    Ask::CharFlush();
    printf(Ask::CharReady() ? "characters ready\n" : "no characters ready\n");
}


int main() {
    IDEConsole::StdoutNoBuffering();
 
    if ( Ask::YesNo("Test Ask::Int", true) ) {
        test_int();
	}

    if ( Ask::YesNo("Test Ask::Hex", true) ) {
        test_hex();
	}

    if ( Ask::YesNo("Test Ask::Float", true) ) {
        test_float();
	}

    if ( Ask::YesNo("Test Ask::Double", true) ) {
        test_double();
	}

    if ( Ask::YesNo("Test Ask::Char", true) ) {
        test_char();
	}

    if ( Ask::YesNo("Test Ask::String", true) ) {
        test_string();
	}

    if ( Ask::YesNo("Test Ask::YesNo", true) ) {
        test_yesno();
	}

    if ( Ask::YesNo("Test Ask::Pause", true) ) {
        test_pause();
	}

    if ( Ask::YesNo("Test Ask::CharReady and Ask::CharFlush", true) ) {
        test_charReadyFlush();
	}
}
