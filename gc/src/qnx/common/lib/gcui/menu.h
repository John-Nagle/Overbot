/////////////////////////////////////////////////////////////////////////////
//
//    File: menu.h
//
//    Usage:
//        #include "menu.h"
//
//        class Object {
//        ...
//        };
//
//        class ObjectMenu: public Menu {
//        public:
//            ObjectMenu(Object *object, char *description, 
//                       char selectChar, char defaultChar):
//                Menu(description, selectChar, defaultChar)
//                {
//                    o = object;
//                }
//            void Display();
//            bool Process(char c);
//        private:
//            Object *o;  // object to which this menu applies
//        };
//
//        void ObjectMenu::Display()
//        {
//            printf("  x - display each option like this\n");
//            ...
//        }
//
//        bool ObjectMenu::Process(char c)
//        {
//            switch (c) {
//                case 'x':
//                    // action for 'x' here
//                    break;
//                ...
//                default:
//                    // option not found
//                    return false;
//                    break;
//            }
// 
//            return true;
//        }
//
//        ObjectMenu m("Object Menu", 'm', 'h');
//        MenuHandler mh;
//
//        mh.Install(&m);
//        mh.Start();
//
//    Description:
//        Header file for menu.cc.
//
//        A uniform interface for interacting with the user through
//        menus.  A flat menu system.  The user is either shown a list
//        of menus to choose from, or a list of options corresponding to 
//        the menu last selected.
//
//        Each menu is associated with a selection character.  If more than
//        one menu uses that selection character, a number is assigned to
//        the selection character.
//
//        By default, 'Q' is a selection character that allows the user
//        to quit the menu, and 'q' is an option on the submenus that
//        allows the user to go to the main list of menus.
//
//        Menu is a base class.  Users of Menu should define derive a class
//        from the Menu class, and define Display() and Process() routines
//        for the derived class.  The MenuHandler is used to install menus
//        of all types (those derived from Menu) and to start the menuing
//        process.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef MENU_H
#define MENU_H

// use doubly linked list since no push_back() in singly linked list library!
#include <list>
#include <stdio.h>

using namespace std;

#define MENU_NAME_LEN (82)
#define MENU_SELECT_LEN (10)

class Menu {
public:
    Menu(char *description, char selectChar, char defaultChar);
    virtual void Display();
    virtual bool Process(char c);
    char *NameGet();				// the menu handler needs access to
    char SelectGet();				// these routines
    char DefaultGet();
    void IndexSet(int i);
    int IndexGet();
    void LastSet(bool b);
    bool LastGet();
private:
    char name[MENU_NAME_LEN];
    char sel;
    char def;
    int ind;
    bool last;
};

// list and list iterator types are menu pointers so polymorphism will work
class MenuHandler {
public:
    void Install(Menu *m);
    void Start();
private:
    list<Menu *> l;
    list<Menu *>::iterator li;
    void defaultSelection(char *def);
    void menuListDisplay();
};


#endif // MENU_H
