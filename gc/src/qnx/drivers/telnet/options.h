//
//	Option parsing engine for Telnet options
//
//	J. Nagle
//	Team Overbot
//	December, 2003
//
#ifndef OPTIONS_H
#define OPTIONS_H

#include <inttypes.h>
#include <string>
//
//	Table commands - must be greater than 255.
//
const uint32_t k_readsignaturetext = 1001;
const uint32_t k_readvalue = 1002;
//
//	The option table
//
extern const uint32_t* telnetsuboptions[];				// external table
//
//	class OptionEngine  -- table driven suboption recognition engine
//
class OptionEngine {
private:
	std::string m_stringarg;												// a string argument
	bool m_stringargesc;											// last char was terminating char, may be in escape
	int m_optionnum;													// number of option being parsed (-1 if none)
	int m_optionpos;													// position within the option 
public:
	OptionEngine();
	virtual ~OptionEngine();
	void reset();															// reset to ground state
	bool accept(uint8_t code);									// input a char in option mode. Returns false if no more option
protected:
	//	Derived class must redefine to receive option content
	virtual void deliveroption(uint8_t option, uint8_t suboption, uint32_t numarg, const char* stringarg) = 0;
private:
	void addtostringarg(uint8_t code);
	void addtonumarg(uint8_t code);
};

#endif //