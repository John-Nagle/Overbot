//
//	spawnfix.h  --  workaround for const problem with spawn call
//
//	J. Nagle
//	Team Overbot
//	August, 2003
//
#ifndef SPAWNFIX_H
#define SPAWNFIX_H
#include <spawn.h>
//
//	spawn  --  spawn a new process
//
//	Constness problem workaround.
//
inline pid_t spawn( const char* path, int fd_count,  const int fd_map[ ], const struct inheritance* inherit, 
	const char* const argv[ ], const char* const envp[ ])
{	return(spawn(path,fd_count,fd_map,inherit,const_cast<char*const*>(argv),const_cast<char*const*>(envp))); }
		             
#endif // SPAWNFIX_H