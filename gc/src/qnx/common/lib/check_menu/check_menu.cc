/////////////////////////////////////////////////////////////////////////////
//
//    File: check_menu.cc
//
//    Usage:
//        check_menu
//
//    Description:
//        A test file for menu.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July 2003
//
////////////////////////////////////////////////////////////////////////////

#include <list>
#include <stdio.h>
#include "menu.h"

class Object {
public:
    Object();
    void PrintI();
private:
    int i;
};

class ObjectMenu: public Menu {
public:
    ObjectMenu(Object *object, char *description, char selectChar, 
               char defaultChar):
        Menu(description, selectChar, defaultChar)
        {
            o = object;
		}
    void Display();
    bool Process(char c);
private:
    Object *o;
};

Object::Object()
{
    i = 42;
}

void Object::PrintI()
{
    printf("The value of 'i' is '%i'\n", i);
}

void ObjectMenu::Display()
{
    printf("  h - print 'Hello world!'\n");
    printf("  p - print '12357...'\n");
    printf("  c - calculate 2 * 3 * 4\n");
    printf("  i - print the value of i\n");
}

bool ObjectMenu::Process(char c)
{
    switch (c) {
        case 'h':
            printf("Hello world!\n");
            break;
		case 'p':
		    printf("12357...\n");
		    break;
		case 'c':
		    printf("2 * 3 * 4 = %i\n", 2*3*4);
		    break;
		case 'i':
		    o->PrintI();
		    break;
		default:
		    return false;
		    break;
	}

	return true;
}

int main(void)
{
    Object o;
    
    	setvbuf(stdout, NULL,_IOLBF, 0);

    ObjectMenu m1(&o, "Does nothing", 'n', 'h');
    ObjectMenu m2(&o, "Does nothing 2", 'n', 'h');
    Menu m3("Default menu", 'd', 'd');
    ObjectMenu m4(&o, "Does nothing 3", 'n', 'h');
    ObjectMenu m5(&o, "Does not much 1", 'c', 'h');
    ObjectMenu m6(&o, "Does not much 2", 'c', 'h');
    MenuHandler mh;

    mh.Install(&m1);
    mh.Install(&m2);
    mh.Install(&m3);
    mh.Install(&m4);
    mh.Install(&m5);
    mh.Install(&m6);
    mh.Start();
}
