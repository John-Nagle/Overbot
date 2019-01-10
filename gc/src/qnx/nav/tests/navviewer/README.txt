Navigation viewer

26 JAN 2004 -- J. Nagle

The initial version works, but logging is in a rather crude state.  
Cell info logging needs to be installed; right now, it's just drawing
what OpenSteer would draw, which results in huge log files and
slows down OpenSteer. This has to be fixed when we start logging
real cell data, or the system will choke.

Display of LIDAR data is not being done at all, although there is
a window for displaying it.