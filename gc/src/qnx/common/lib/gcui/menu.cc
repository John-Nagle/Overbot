/////////////////////////////////////////////////////////////////////////////
//
//    File: menu.cc
//
//    Usage:
//        see menu.h
//
//    Description:
//        see menu.h
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July, 2003
//
/////////////////////////////////////////////////////////////////////////////

#include "menu.h"
#include "ask.h"

//
//    Menu::Menu - default constructor for Menu class
//
Menu::Menu(char *description, char selectChar, char defaultChar)
{
    // set up parameter values
    strncpy(name, description, MENU_NAME_LEN);
    name[MENU_NAME_LEN-1] = '\0';
    sel = selectChar;
    def = defaultChar;

    // set up default values
    ind = 1;
    last = true;
}

//
//    Menu::Display - display the options available for this menu
//
//    This routine should be replaced with Display() for a derived class.
//    See "usage" above.
//
//    Returns nothing
//
void Menu::Display()
{
    printf("No options menu installed.\n");
}

//
//    Menu::Process - process the options available for this menu
//
//    This routine should be replaced with Process() for a derived class.
//    See "usage" above.
//
//    Returns true if the option was processed, false if not
//
bool Menu::Process(char c)
{
    printf("No options processing installed.\n");

    return false;
}

//
//    Menu::NameGet - get the name for this menu
//
//    Returns the menu name
//
char *Menu::NameGet()
{
    return name;
}

//
//    Menu::SelectGet - get the selection character for this menu
//
//    Returns the selection character
//
char Menu::SelectGet()
{
    return sel;
}

//
//    Menu::DefaultGet - get the default character for this menu
//
//    This is the default selection when the menu is first displayed.
//
//    Returns the default character
//
char Menu::DefaultGet()
{
    return def;
}

//
//    Menu::IndexSet - set the index for this menu
//
//    Indicies are used to distinguish menus that have the same selection
//    character.
//
//    Returns nothing
//
void Menu::IndexSet(int i)
{
    ind = i;
}

//
//    Menu::IndexGet - get the index for this menu
//
//    Indicies are used to distinguish menus that have the same selection
//    character.
//
//    Returns the index
//
int Menu::IndexGet()
{
    return ind;
}

//
//    Menu::LastSet - set last for this menu
//
//    The last data member is used to designate if this menu is the last
//    menu in the list using this select character.
//
//    Returns nothing
//
void Menu::LastSet(bool b)
{
    last = b;
}

//
//    Menu::LastGet - get last for this menu
//
//    The last data member is used to designate if this menu is the last
//    menu in the list using this select character.
//
//    Returns value of last
//
bool Menu::LastGet()
{
    return last;
}

//
//    MenuHandler::Install - install a menu
//
//    Need to pass in a pointer to a menu since polymorphism is used
//    with the Display() and Process() routines.
// 
//    Returns nothing
//
void MenuHandler::Install(Menu *m)
{
    char selNew;

    // if there are other menus with the same select character,
    // set the proper values of index and last
    selNew = m->SelectGet();
    for ( li = l.begin() ; li != l.end() ; li++ ) {
        if ( ((*li)->SelectGet() == selNew) && ((*li)->LastGet()) ) {
            // found last menu with the same select character

            // current menu is no longer last with same select char
            (*li)->LastSet(false);

            // new menu is last with same select char
            m->LastSet(true);

            // new menu index needs to be one more than current menu
            m->IndexSet((*li)->IndexGet() + 1);
		}
	}

	// push onto end of menu list
    l.push_back(m);
}

//
//    MenuHandler::Start - start the menu handler
//
//    Returns nothing
//
void MenuHandler::Start()
{
    char defStr[MENU_SELECT_LEN];
    char defChar;
    char sel;
    int ind;
    int matches;

    defaultSelection(defStr);
    do {
        // display available menus and prompt user for selection
		printf("\n");
		printf("Menu List:\n");
        menuListDisplay();
		printf("\n");
		printf("  Type 'Q' to quit\n");
		printf("\n");
        Ask::String("Select a menu", defStr, defStr, sizeof(defStr));
        matches = sscanf(defStr, "%c%i", &sel, &ind);
        
        // handle 'Q' case
        if ( sel == 'Q' ) {
        	return;
        }

        // find corresponding menu
        li = l.begin();
        while ( li != l.end() ) {
            if ( (*li)->SelectGet() == sel ) {
                if ( (matches == 1) && ((*li)->IndexGet() == 1) &&
                     ((*li)->LastGet() == true) ) {
                    // found it, only menu with this select char
                    break;
				} else if ( (matches == 2) && ((*li)->IndexGet() == ind) ) {
				    // found it, menu with proper select char and index
				    break;
				}
            }
            li++;
        } 

        // deal with options
        if ( li != l.end() ) {
			// display menu options and prompt user for selection
			defChar = (*li)->DefaultGet();
			do {
				printf("\n");
			    printf("%s Options List:\n", (*li)->NameGet());
				(*li)->Display();
				printf("\n");
				printf("  Type 'q' to go to the main menu\n");
				printf("\n");
				defChar = Ask::Char("Select a command", defChar);
				if ( defChar != 'q' ) {
				    if ( !(*li)->Process(defChar) ) {
						printf("\007Option '%c' not available.\n", defChar);
					}
				}
			} while ( defChar != 'q' );
		} else {
		    // came to the end of the list without a match
		    printf("\007Menu '%c' not available.\n", sel);
		}
    } while ( 1 );
}

//
//    MenuHandler::defaultSelection - find the default selection string
//
//    A string is used rather than a character because the index may be
//    needed. 
//
//    Returns nothing
//
void MenuHandler::defaultSelection(char *defStr)
{
    // default selection is the first installed menu
    li = l.begin();
    sprintf(defStr, "%c", (*li)->SelectGet());

    // if not last menu for that select char, append index
    // if last menu, index will be zero since this is first menu installed
    if ( !(*li)->LastGet() ) {
        sprintf(defStr, "%s%i", defStr, (*li)->IndexGet());
	}
}

//
//    MenuHandler::menuListDisplay - display a list of the menus available
//
//    Returns nothing
//
void MenuHandler::menuListDisplay()
{
    char ind[MENU_SELECT_LEN];

    // go through the list of menus
    for ( li = l.begin() ; li != l.end() ; li++ ) {
        if ( ((*li)->IndexGet() == 1) && ((*li)->LastGet()) ) {
            // first and last, don't include index
            sprintf(ind, " ");
        } else {
            // not first and last, include index
            sprintf(ind, "%i", (*li)->IndexGet());
		}
        printf("  %c%s - %s\n", (*li)->SelectGet(), ind, (*li)->NameGet());
    }
}
