#!BPY

""" Registration info for Blender menus:
Name: 'Raw Overbot LIDAR (.log)...'
Blender: 235
Group: 'Import'
Tip: 'Import a raw Overbot LIDAR (.log) file.'
"""

__author__ = "John Nagle"


__bpydoc__ = """\
This script imports raw Overbot LIDAR log files into Blender.

This version assumes that all the data was taken with the scanner stationary,
but tilting.
"""

import Blender
#	Begin actual import filter
#	***MORE***
class RawLIDARlogImport:

	def __init__(self, filename):
		global errmsg
		print "Trying to import raw LIDAR data from %s ..." % filename
		self.i = 0
		errmsg = ''
		self.importdir = Blender.sys.dirname(filename)
		try:
			file = open(filename, 'r')
		except IOError, (errno, strerror):
			errmsg = "IOError #%s: %s" % (errno, strerror)
			print errmsg
			return None
		errmsg = ''
		print "Opened filename: ",filename
		#	***MORE***
		file.close()
        

#	Link into Blender GUI


def filesel_callback(filename):
  test = RawLIDARlogImport(filename)

Blender.Window.FileSelector(filesel_callback, "Import raw Overbot LIDAR", "*.log")