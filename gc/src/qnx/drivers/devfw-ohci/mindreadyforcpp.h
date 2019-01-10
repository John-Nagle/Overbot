//
//	mindeadyforcpp.h  -- include Mindready lla.h safely under C++
//
//	Under gcc 2.9.x, in C++ code, sizeof(bool) = 4.
//	But in C code, in <stdbool.h>, bool is defined as char.
//	This causes major structure incompatibilities.
//	So we have this workaround file.
//
//
//	Copyright 2005 by John Nagle
//	999 Woodland Avenue
//	Menlo Park, CA  94025
//
//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.

//	You should have received a copy of the GNU General Public License
//	along with this program; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#ifndef MINDREADYFORCPP_H
#define MINDREADYFORCPP_H
#ifdef __cplusplus													// only if compiling C++ code
#ifdef  __INCllah													// if already included lla.h
#error "mindeadyforcpp.h must be included before lla.h".	// out of sequence includes
#endif 
#ifdef bool
#error "bool is defined as a preprocessor definition in C++, and should not be."
#endif
#define bool char												// define bool as char for following includes only
#define _cplusplus												// ***WORKAROUND for bad macro in lla.h ***
#else
#error "llaforcpp.h is only for use in C++ programs"
#endif // __cplusplus

//	Include Mindready files.
//	Mindready files must be included here, only.
#include "lla.h"
#include "dbuf.h"

//	Undo our wierd definition of "bool"
#ifdef __cplusplus													// only if compiling C++ code
#undef bool
#endif // __cplusplus
#endif // MINDREADYFORCPP_H