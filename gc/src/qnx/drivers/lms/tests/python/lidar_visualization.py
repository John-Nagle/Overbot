# -----------------------------------------------------------------
#
#  lidar_visualization.py  --  read a LIDAR log file and visualize as a mesh
#
#struct LidarScanLineHeader {
#  uint64_t  m_timestamp;        // CLOCK_REALTIME, in nanoseconds
#  float     m_tilt;             // tilt angle of unit. 0=straight down, pi/2=straight ahead
#  uint8_t   m_sensorid;         // which LMS (future expansion)
#  uint8_t   m_statusByte;       // LMS status byte
#  uint8_t   m_scanIndex;        // scan index (cyclic)
#  uint8_t   m_unused1;          // (future expansion)
#  uint16_t  m_unused2;          // (future expansiion, and fill to word)
#  uint16_t  m_valueCount;       // count of m_range
#};
#
#struct LidarScanLine {
#  LidarScanLineHeader m_header;      // the header
#  uint16_t  m_range[LMS_MAX_DATA_POINTS];// depth data (m_valuecount entries)
#};
#
# -----------------------------------------------------------------

# -----------------------------------------------------------------
# TODO
# bug: gui text color is unstable
# feature: automatically switch to 3d view after building the mesh
# feature: change ray projection to a matrix transformation and include pose
# -----------------------------------------------------------------

FORWARD_MOTION=0.0
IGNORE_LONG_EDGES=1.0

import struct

import math
from math import *

# -----------------------------------------------------------------
# 3d coordinate system (right handed)
# x is forward
# y is left
# z is up
# theta is rotations in the xy plane, x axis is zero (forward)
# tilt is rotations in the zx plane, z axis is zero (up)
# -----------------------------------------------------------------

# -----------------------------------------------------------------
class lidar_ray:
# -----------------------------------------------------------------
  '''this class stores the orientation of a lidar ray and projects it into a 3D 4-tuple'''
  theta=0.0
  tilt=0.0
  value=0
  pose=0.0
  scaling=0.01   # convert scanner centimeters to meters

  def set_theta(self,theta):
    self.theta=theta

  def set_tilt(self,tilt):
    self.tilt=tilt

  def set_value(self,value):
    self.value=value

  def set_pose(self,pose):
    # fmylb, add gps pose
    self.pose=pose

  def projection(self):
    # fmylb, handle error conditions
    # temporary pose to move the scanner forward
    k=self.value*self.scaling
    v=( self.value<8183,                    # valid
        self.pose+cos(self.theta)*sin(self.tilt)*k,   # x
        sin(self.theta)*k,                  # y
        cos(self.theta)*cos(self.tilt)*k    # z
      )
    return(v)

# -----------------------------------------------------------------
class lidar_visualization:
# -----------------------------------------------------------------
  '''Reads a lidar log and generates a mesh of tristrips. This code is highly specific. If the log file format changes this will break'''
  global FORWARD_MOTION

  HeaderFormat = "LLfBBBBHHI"                    # format of header part
  HeaderFormatL = struct.calcsize(HeaderFormat)  # header size (must be 24)
  DataFormat = "H"                               # format of one data item
  DataFormatL = struct.calcsize(DataFormat)      # data item size
  TotalRecordL = 392                             # entire record, including filler
  DataRecordL = TotalRecordL-HeaderFormatL       # data part, including filler

  # scanner is 180 degrees right-to-left in 1 degree increments
  START_ANGLE=(-3.1415926/2.0)
  ANGLE_STEP=(2.0*3.1415926/360.0)

  # simulate forward motion for each scan line

  def build_mesh(self,infile,tilt_ignore):
    '''Read a log file and convert to a mesh of tristrips'''

    mesh=create_mesh()

    # place a half-meter cube to represent the scanner
    verts=cube_tristrip(0.5)
    add_tristrip_to_mesh(mesh,verts)

    first_scanline=1
    num_scanline=0

    pose1=0.0
    pose2=0.0

    # begin reading all scanlines
    while 1:
      buf=infile.read(self.HeaderFormatL)
      if(len(buf)==0): break
      num_scanline=num_scanline+1
      pose2=pose2+FORWARD_MOTION
      header2=struct.unpack(self.HeaderFormat,buf)
      time2=(header2[0]*0.000000001)+(header2[1]*(4.294967296))
      n2=header2[8]
      d2=infile.read(self.DataRecordL);
      if(len(d2)==0): break
      if(header2[2]<radians(tilt_ignore)): continue
      if(n2!=181 and n2!=180): continue

      # found a valid scanline

      if(first_scanline):
        # store the first scanline
        first_scanline=0
        header1=header2
        time1=time2
        n1=n2
        d1=d2
        pose1=pose2
        continue

      # fmylb, kludge
#      if(time2-time1<0.005): continue

      # build a tristrip using two scanlines

      verts=[]
      ray1=lidar_ray()
      ray1.set_tilt(3.1415926-header1[2])     # scanner tilt is down
      ray1.set_pose(pose1)
      ray2=lidar_ray()
      ray2.set_tilt(3.1415926-header2[2])     # scanner tilt is down
      ray2.set_pose(pose2)

      if(n1==n2):
        # equal length scanlines
        if(n1==181): theta=self.START_ANGLE
        else: theta=self.START_ANGLE+0.5*self.ANGLE_STEP

        for i in range(0,n1):
          data=d1[i*self.DataFormatL:(i+1)*self.DataFormatL]
          ray1.set_theta(theta)
          ray1.set_value(struct.unpack(self.DataFormat,data)[0])
          verts.append(ray1.projection())

          data=d2[i*self.DataFormatL:(i+1)*self.DataFormatL]
          ray2.set_theta(theta)
          ray2.set_value(struct.unpack(self.DataFormat,data)[0])
          verts.append(ray2.projection())

          theta+=self.ANGLE_STEP
      elif(n1>n2):
        # bottom scanline is longer, top scanline is offset by a half-step
        theta1=self.START_ANGLE
        theta2=self.START_ANGLE+0.5*self.ANGLE_STEP

        for i in range(0,n2):
          data=d1[i*self.DataFormatL:(i+1)*self.DataFormatL]
          ray1.set_theta(theta1)
          ray1.set_value(struct.unpack(self.DataFormat,data)[0])
          verts.append(ray1.projection())

          data=d2[i*self.DataFormatL:(i+1)*self.DataFormatL]
          ray2.set_theta(theta2)
          ray2.set_value(struct.unpack(self.DataFormat,data)[0])
          verts.append(ray2.projection())

          theta1=theta1+self.ANGLE_STEP
          theta2=theta2+self.ANGLE_STEP

        i=n1-1
        data=d1[i*self.DataFormatL:(i+1)*self.DataFormatL]
        ray1.set_theta(theta1)
        ray1.set_value(struct.unpack(self.DataFormat,data)[0])
        verts.append(ray1.projection())
      else:
        # top scanline is longer, bottom scanline is offset by a half-step
        theta1=self.START_ANGLE+0.5*self.ANGLE_STEP
        theta2=self.START_ANGLE

        i=0
        data=d2[i*self.DataFormatL:(i+1)*self.DataFormatL]
        ray2.set_theta(theta2)
        ray2.set_value(struct.unpack(self.DataFormat,data)[0])
        verts.append(ray2.projection())
        theta2=theta2+self.ANGLE_STEP

        for i in range(0,n1):
          data=d2[(i+1)*self.DataFormatL:(i+2)*self.DataFormatL]
          ray2.set_theta(theta2)
          ray2.set_value(struct.unpack(self.DataFormat,data)[0])
          verts.append(ray2.projection())

          data=d1[i*self.DataFormatL:(i+1)*self.DataFormatL]
          ray1.set_theta(theta1)
          ray1.set_value(struct.unpack(self.DataFormat,data)[0])
          verts.append(ray1.projection())

          theta1=theta1+self.ANGLE_STEP
          theta2=theta2+self.ANGLE_STEP

      add_tristrip_to_mesh(mesh,verts)

      header1=header2
      time1=time2
      n1=n2
      d1=d2
      pose1=pose2

    print "mesh done"
    print "read %d scanlines" % num_scanline
    draw_mesh(mesh)

# -----------------------------------------------------------------
# utility fns
# -----------------------------------------------------------------

def cube_tristrip(scale):
  '''builds a cube out of a single tristrip'''
  verts=[]
  r=0.5*scale
  v=(True,r,r,r)
  verts.append(v)
  v=(True,r,-r,r)
  verts.append(v)
  v=(True,-r,r,r)
  verts.append(v)
  v=(True,-r,-r,r)
  verts.append(v)
  v=(True,-r,r,-r)
  verts.append(v)
  v=(True,-r,-r,-r)
  verts.append(v)
  v=(True,r,r,-r)
  verts.append(v)
  v=(True,r,-r,-r)
  verts.append(v)
  v=(True,r,r,r)
  verts.append(v)
  v=(True,r,-r,r)
  verts.append(v)
  v=(False,r,r,r)
  verts.append(v)
  v=(True,-r,r,r)
  verts.append(v)
  v=(True,r,r,r)
  verts.append(v)
  v=(True,-r,r,-r)
  verts.append(v)
  v=(True,r,r,-r)
  verts.append(v)
  v=(False,r,r,r)
  verts.append(v)
  v=(True,-r,-r,-r)
  verts.append(v)
  v=(True,r,-r,-r)
  verts.append(v)
  v=(True,-r,-r,r)
  verts.append(v)
  v=(True,r,-r,r)
  verts.append(v)
  return(verts)

def max_edge_length(v0,v1,v2):
  '''returns the length of the longest edge of a tri'''
  dx=v0[1]-v1[1]
  dy=v0[2]-v1[2]
  dz=v0[3]-v1[3]
  d2=dx*dx+dy*dy+dz*dz
  dx=v0[1]-v2[1]
  dy=v0[2]-v2[2]
  dz=v0[3]-v2[3]
  d2=max(d2,dx*dx+dy*dy+dz*dz)
  dx=v1[1]-v2[1]
  dy=v1[2]-v2[2]
  dz=v1[3]-v2[3]
  d2=max(d2,dx*dx+dy*dy+dz*dz)
  return(sqrt(d2))

# -----------------------------------------------------------------
# Blender specific code begins here
# Do not taint the code above here with Blender stuff!
# -----------------------------------------------------------------

import Blender
from Blender import NMesh

def create_mesh():
  '''creates or replaces the mesh'''
  mesh=NMesh.GetRaw("overbot_lidar_mesh")
  if(mesh==None): mesh=NMesh.New("overbot_lidar_mesh")
  mesh.verts=[]
  mesh.faces=[]
  return(mesh)
  
def add_tristrip_to_mesh(mesh,verts):
  '''verts is an array of 4-tuple of (valid,x,y,z) where valid is True to indicate the vertex is valid. n verts will create n-2 tris.'''

  global IGNORE_LONG_EDGES
  
  n=len(verts)
  for i in range(0,n):
    # blender has x to the right, y forward, z up
    v=NMesh.Vert(-verts[i][2],verts[i][1],verts[i][3])
    mesh.verts.append(v)
    # generate a tri if the 3 verts are valid
    if(i>1 and verts[i-2][0] and verts[i-1][0] and verts[i-0][0]):
      # discard tris whose longest edge exceeds 1 meter
      l=max_edge_length(verts[i-2],verts[i-1],verts[i-0])
      if(l<IGNORE_LONG_EDGES):
        f=NMesh.Face()
        f.v.append(mesh.verts[-3]);
        f.v.append(mesh.verts[-2]);
        f.v.append(mesh.verts[-1]);
        mesh.faces.append(f)

def draw_mesh(mesh):
  '''writes the mesh data into Blender'''
  print "mesh has %d verts %d faces" % (len(mesh.verts),len(mesh.faces))
  obj=NMesh.PutRaw(mesh,"overbot_lidar_mesh")
  Blender.Redraw()
  
class lidar_import:
  '''the blender file i/o part of importing'''
  def __init__(self,filename,tilt_ignore):
    global errmsg
    print "Trying to import raw LIDAR data from %s ..." % filename
    errmsg=''
    self.importdir=Blender.sys.dirname(filename)
    try:
      file=open(filename,"rb")
    except IOError,(errno,strerror):
      errmsg="IOError #%s: %s" % (errno,strerror)
      print errmsg
      return
    errmsg=''
    obj=lidar_visualization()
    obj.build_mesh(file,tilt_ignore)
    file.close()
    print "Success"

# -----------------------------------------------------------------
# Blender GUI
# -----------------------------------------------------------------

from Blender.BGL import *
from Blender.Draw import *

# each of these corresponds to one of the ui elements
EVENT_TILT_IGNORE=1
EVENT_OPEN=2
EVENT_RELOAD=3
EVENT_EXIT=4

working_file=''
tilt_ignore=20
tilt_ignore_button=Create(20)

def lidar_import_callback(filename):
  '''this is called after selecting something from the file dialog'''
  global working_file
  global tilt_ignore

  working_file=filename
  lidar=lidar_import(working_file,tilt_ignore)
  Blender.Redraw()

def draw_event():
  '''blender calls this to draw the scripts window, put Blender.Draw and Blender.BGL stuff here'''
  global EVENT_TILT_IGNORE,EVENT_OPEN,EVENT_RELOAD,EVENT_EXIT
  global working_file
  global tilt_ignore
  global tilt_ignore_button

  # some opengl stuff to draw text
  glClear(GL_COLOR_BUFFER_BIT)
  glRasterPos2d(8,133)
  Text("Overbot LIDAR Visualization")
  glRasterPos2d(8,93)
  Text("Current Working File:")
  glRasterPos2d(18,73)
  Text(working_file)

  # the ui elements
  tilt_ignore_button=Number("Tilt Ignore",EVENT_TILT_IGNORE,8,43,150,18,tilt_ignore,0,90,"Set the number of degrees of tilt ignored when loading. Press Reload after changing this value!")
  PushButton("Open",EVENT_OPEN,10,10,80,18,"Select a file and set it as the current working file")
  PushButton("Reload",EVENT_RELOAD,140,10,80,18,"Reload the current working file")
  PushButton("Exit",EVENT_EXIT,270,10,80,18,"Terminate the script (ESC)")

def event(ev,val):
  '''catch Blender.Draw keyboard events here'''
  if(ev==ESCKEY and val==0):
    Exit()

def button_event(ev):
  '''catch ui element events here'''
  global EVENT_TILT_IGNORE,EVENT_OPEN,EVENT_RELOAD,EVENT_EXIT
  global opendialog_file
  global working_file
  global tilt_ignore
  global tilt_ignore_button

  if(ev==EVENT_TILT_IGNORE):
    tilt_ignore=tilt_ignore_button.val
  if(ev==EVENT_OPEN):
    Blender.Window.FileSelector(lidar_import_callback,"Import Overbot LIDAR","*.log")
  if(ev==EVENT_RELOAD):
    lidar=lidar_import(working_file,tilt_ignore)
    Blender.Redraw()
  if(ev==EVENT_EXIT):
    Exit()
    
# this begins the blender ui event loop
# probably not a good idea to put code after here
Register(draw_event,event,button_event)

