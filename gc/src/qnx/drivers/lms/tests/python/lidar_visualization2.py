# -----------------------------------------------------------------
#
#  lidar_visualization.py  --  read a LIDAR log file and visualize as a mesh
#
#
# -----------------------------------------------------------------

# -----------------------------------------------------------------
# TODO
# bug: gui text color is unstable
# bug: scanlines are not being interlaced
# feature: automatically switch to 3d view after building the mesh
# feature: change ray projection to a matrix transformation and include pose
# -----------------------------------------------------------------

IGNORE_LONG_EDGES=2.0
TIME_STRING_FORMAT='%Y-%m-%d %H:%M:%S' # time format example, 2005-07-15 16:21:01
MAX_BLENDER_X = 30
MAX_BLENDER_Y = 30

# up to 618
FORCE_GPS_Z_ZERO=1
FORCE_SMOOTH_LIDAR_TIME=0
ANIMATION_DRAW=0
DEFAULT_COL=[255,255,255,0] # white

import struct

import lidar_data
from lidar_data import *
import math
from math import *

# -----------------------------------------------------------------
class lidar_visualization:
# -----------------------------------------------------------------
  '''Reads a lidar log and generates a mesh of tristrips. This code is highly specific. If the log file format changes this will break'''
  HeaderFormat = "LLfBBBBHHI"                    # format of header part
  HeaderFormatL = struct.calcsize(HeaderFormat)  # header size (must be 24)
  DataFormat = "H"                               # format of one data item
  DataFormatL = struct.calcsize(DataFormat)      # data item size
  TotalRecordL = 392                             # entire record, including filler
  DataRecordL = TotalRecordL-HeaderFormatL       # data part, including filler

  # scanner is 180 degrees right-to-left in 1 degree increments
  START_ANGLE=(-3.1415926/2.0)
  ANGLE_STEP=(2.0*3.1415926/360.0)

  def build_mesh(self,infile,tilt_ignore,gps,bVehiclePos=False):
    '''Read a log file and convert to a mesh of tristrips'''
    global CP

    mesh=create_mesh()
    if (bVehiclePos):
        gpsmesh=create_gps_mesh()

    # place a half-meter cube to represent the scanner
#    verts=cube_tristrip(0.5)
#    add_tristrip_to_mesh(mesh,verts)

    first_scanline=1
    num_scanline=0

    pose1=0.0
    pose2=0.0

    # begin reading all scanlines
    while 1:
      buf=infile.read(self.HeaderFormatL)
      if(len(buf)==0): break
      d2=infile.read(self.DataRecordL);
      if(len(d2)==0): break
      header2=struct.unpack(self.HeaderFormat,buf)
      time2=(header2[0]*0.000000001)+(header2[1]*(4.294967296))
      if(FORCE_SMOOTH_LIDAR_TIME and first_scanline==0):
        time2=time1+1.0/75.0
      if(time2<CP.startTime): continue
      if(time2>CP.endTime): break
      num_scanline=num_scanline+1
      pose2=gps.pose(time2)
      gpserror=gps.currentError()
      n2=header2[8]
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
      if (bVehiclePos):
        gpsverts=get_gps_verts(pose1, pose2, gpserror)

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
          data=d1[i*self.DataFormatL:(i+1)*self.DataFormatL]
          ray1.set_theta(theta1)
          ray1.set_value(struct.unpack(self.DataFormat,data)[0])
          verts.append(ray1.projection())

          data=d2[(i+1)*self.DataFormatL:(i+2)*self.DataFormatL]
          ray2.set_theta(theta2)
          ray2.set_value(struct.unpack(self.DataFormat,data)[0])
          verts.append(ray2.projection())

          theta1=theta1+self.ANGLE_STEP
          theta2=theta2+self.ANGLE_STEP

      add_tristrip_to_mesh(mesh,verts)
      if (bVehiclePos):
        add_gps_to_mesh(gpsmesh, gpsverts)

      header1=header2
      time1=time2
      n1=n2
      d1=d2
      pose1=pose2

    print "mesh done"
    print "read %d scanlines" % num_scanline
    gps.dump()
    if (bVehiclePos):
      draw_gps_mesh(gpsmesh)

    draw_mesh(mesh)

  def build_mesh_v1(self,lidar,gps,bVehiclePos=False):
    '''Read a log file and convert to a mesh of tristrips'''

    if isinstance(lidar,BackwardFilter):
      # data from backward filter can be (x,y,z) format. for speed,
      # we need to call other function to build mesh
      return self.build_mesh_from_backwardFilteredData(lidar,gps,bVehiclePos)
        
    mesh=create_mesh()
    if (bVehiclePos):
        gpsmesh=create_gps_mesh()
   
    pose1=0.0
    pose2=0.0
    num_scanline=0
    ray1=lidar_ray()
    ray2=lidar_ray()
    
    # begin reading all scanlines
    for line2 in lidar:
      # line2 has [time,tilt,rangeCount,[ranges]]
      pose2=gps.pose(line2[0])
      gpserror=gps.currentError()
      num_scanline += 1
      if(num_scanline == 1):
        # store the first scanline
        line1 = line2
        pose1 = pose2
        continue
      
      # build a tristrip using two scanlines
      verts=[]
      if (bVehiclePos):
        gpsverts=get_gps_verts(pose1, pose2, gpserror)

      n1 = line1[2]
      n2 = line2[2]
        
#      ray1.set_tilt(3.1415926-line1[1])     # scanner tilt is down
      ray1.set_tilt(line1[1])     # scanner tilt is down
      ray1.set_pose(pose1)
#      ray2.set_tilt(3.1415926-line2[1])     # scanner tilt is down
      ray2.set_tilt(line2[1])     # scanner tilt is down
      ray2.set_pose(pose2)

      if(n1==n2):
        # equal length scanlines
        if(n1==181): theta=self.START_ANGLE
        else: theta=self.START_ANGLE+0.5*self.ANGLE_STEP

        for i in range(0,n1):
          ray1.set_theta(theta)
          ray1.set_value(line1[3][i])
          verts.append(ray1.projection())

          ray2.set_theta(theta)
          ray2.set_value(line2[3][i])
          verts.append(ray2.projection())

          theta+=self.ANGLE_STEP
          
      elif(n1>n2):
        # bottom scanline is longer, top scanline is offset by a half-step
        theta1=self.START_ANGLE
        theta2=self.START_ANGLE+0.5*self.ANGLE_STEP

        for i in range(0,n2):
          ray1.set_theta(theta1)
          ray1.set_value(line1[3][i])
          verts.append(ray1.projection())

          ray2.set_theta(theta2)
          ray2.set_value(line2[3][i])
          verts.append(ray2.projection())

          theta1=theta1+self.ANGLE_STEP
          theta2=theta2+self.ANGLE_STEP

        i=n1-1
        ray1.set_theta(theta1)
        ray1.set_value(line1[3][i])
        verts.append(ray1.projection())
      else:
        # top scanline is longer, bottom scanline is offset by a half-step
        theta1=self.START_ANGLE+0.5*self.ANGLE_STEP
        theta2=self.START_ANGLE

        i=0
        ray2.set_theta(theta2)
        ray2.set_value(line2[3][i])
        verts.append(ray2.projection())
        theta2=theta2+self.ANGLE_STEP
        
        for i in range(0,n1):
          ray1.set_theta(theta1)
          ray1.set_value(line1[3][i])
          verts.append(ray1.projection())

          ray2.set_theta(theta2)
          ray2.set_value(line2[3][i+1])
          verts.append(ray2.projection())

          theta1=theta1+self.ANGLE_STEP
          theta2=theta2+self.ANGLE_STEP

      add_tristrip_to_mesh(mesh,verts)
      if (bVehiclePos and gpsverts != []):
        add_gps_to_mesh(gpsmesh, gpsverts)

      line1 = line2
      pose1 = pose2

      if ANIMATION_DRAW:
        draw_mesh(mesh)
        if (bVehiclePos):
          draw_gps_mesh(gpsmesh)

        time.sleep(0.025)

    print "mesh done"
    print "read %d scanlines" % num_scanline
    gps.dump()
    if (bVehiclePos):
      draw_gps_mesh(gpsmesh)
      
    draw_mesh(mesh)
    
  def build_mesh_from_backwardFilteredData(self,lidar,gps,bVehiclePos=False):
    '''Read a log file and convert to a mesh of tristrips'''

    mesh=create_mesh()
    if (bVehiclePos):
        gpsmesh=create_gps_mesh()

    bFirstScanLine=True

    # begin reading all scanlines
    while 1:
      # get a valid scan data
      verts2 = lidar.getVerts()
      # getVerts() return (pose,[(True|False,x,y,z),...])
      if verts2 == []:
        # not more data
        break;

      
      n2 = len(verts2[1])
      if(bFirstScanLine):
        # store the first scan data
        bFirstScanLine=0
        verts1=verts2
        n1 = n2
        continue

      # build a tristrip using two scan data
      verts=[]
      if (bVehiclePos):
        gpsverts=get_gps_verts(verts1[0],verts2[0], 0.5)

      if(n1==n2):
        for i in range(0,n1):
          verts.append(verts1[1][i])
          verts.append(verts2[1][i])

      elif(n1>n2):
        # bottom scanline is longer, top scanline is offset by a half-step

        for i in range(0,n2):
          verts.append(verts1[1][i])
          verts.append(verts2[1][i])

        i=n1-1
        verts.append(verts1[1][i])
      else:
        # top scanline is longer, bottom scanline is offset by a half-step

        verts.append(verts2[1][0])

        for i in range(0,n1):
          verts.append(verts1[1][i])
          verts.append(verts2[1][i+1])

      add_tristrip_to_mesh(mesh,verts)
      if (bVehiclePos):
        add_gps_to_mesh(gpsmesh, gpsverts)

      n1=n2
      verts1 = verts2

    print "mesh done"
    print "read ",lidar.getNumOfScanLines()," scanlines,",
    print "invalid line ",lidar.getNumOfInvalidScanLines()
    gps.dump()
    draw_mesh(mesh)
    if (bVehiclePos):
      draw_gps_mesh(gpsmesh)
      
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

def get_gps_verts(pose1, pose2, gpserror):
	verts=[]
	global CP

	x1=pose1[0]
	y1=pose1[1]
	z1=pose1[2]
	x2=pose2[0]
	y2=pose2[1]
	z2=pose2[2]
	
	D=0.5
        if (CP.bGPSerror):
          D=gpserror;
	h = y2-y1
	w = x2-x1
        
	l = sqrt(h*h + w*w)
	if l == 0:
          # the vehicle didn't move
          return verts
        
	sintheta=h/l
	costheta=w/l
	
	v = (True,x2*pose1[3]+y2*pose1[6]+z2*pose1[9],\
                  x2*pose1[4]+y2*pose1[7]+z2*pose1[10],\
                  x2*pose1[5]+y2*pose1[8]+z2*pose1[11])
	verts.append(v)
	v = (True,(x1+D*sintheta)*pose1[3]+(y1-D*costheta)*pose1[6]+z1*pose1[9],\
                  (x1+D*sintheta)*pose1[4]+(y1-D*costheta)*pose1[7]+z1*pose1[10],\
                  (x1+D*sintheta)*pose1[5]+(y1-D*costheta)*pose1[8]+z1*pose1[11])
	verts.append(v)
	v = (True,(x1-D*sintheta)*pose1[3]+(y1+D*costheta)*pose1[6]+z1*pose1[9],\
                  (x1-D*sintheta)*pose1[4]+(y1+D*costheta)*pose1[7]+z1*pose1[10],\
                  (x1-D*sintheta)*pose1[5]+(y1+D*costheta)*pose1[8]+z1*pose1[11])
	verts.append(v)
	
	return(verts)


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

def create_gps_mesh():
  gpsmesh=NMesh.GetRaw("overbot_gps_mesh")
  if(gpsmesh==None): gpsmesh=NMesh.New("overbot_gps_mesh")
  gpsmesh.verts=[]
  gpsmesh.faces=[]
  # enable color
  gpsmesh.hasVertexColours(1)  
  return(gpsmesh)

  
def add_tristrip_to_mesh(mesh,verts):
  '''verts is an array of 4-tuple of (valid,x,y,z) where valid is True to indicate the vertex is valid. n verts will create n-2 tris.'''

  global IGNORE_LONG_EDGES
  
  n=len(verts)
  for i in range(0,n):
    # blender has x to the right, y forward, z up
#    v=NMesh.Vert(-verts[i][2],verts[i][1],verts[i][3])
    v=NMesh.Vert(verts[i][1],verts[i][2],verts[i][3])
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

def add_gps_to_mesh(mesh,verts,color=DEFAULT_COL):
  '''verts is an array of 4-tuple of (valid,x,y,z) where valid is True to indicate the vertex is valid. n verts will create n-2 tris.'''
  
  n=len(verts)
  for i in range(0,n):
    # blender has x to the right, y forward, z up
    v=NMesh.Vert(verts[i][1],verts[i][2],verts[i][3])
    mesh.verts.append(v)
    # generate a tri if the 3 verts are valid
    if(i>1 and verts[i-2][0] and verts[i-1][0] and verts[i-0][0]):
      f=NMesh.Face()
      f.v.append(mesh.verts[-3]);
      f.v.append(mesh.verts[-2]);
      f.v.append(mesh.verts[-1]);
      f.col = [NMesh.Col(color[0],color[1],color[2]),\
               NMesh.Col(color[0],color[1],color[2]),\
               NMesh.Col(color[0],color[1],color[2])]
      f.mode = NMesh.FaceModes['SHAREDCOL']
      mesh.faces.append(f)

        
def draw_mesh(mesh):
  '''writes the mesh data into Blender'''
  print "mesh has %d verts %d faces" % (len(mesh.verts),len(mesh.faces))
  obj=NMesh.PutRaw(mesh,"overbot_lidar_mesh")
  Blender.Redraw()

def draw_gps_mesh(mesh):
  '''writes the mesh data into Blender'''
  print "gps mesh has %d verts %d faces" % (len(mesh.verts),len(mesh.faces))
  obj=NMesh.PutRaw(mesh,"overbot_gps_mesh")
  Blender.Redraw()
  
class lidar_import:
  '''the blender file i/o part of importing'''
  def __init__(self,cp,enableFilters=[],bDrawVehiclePos=False):
    global errmsg
    global MAX_BLENDER_X
    global MAX_BLENDER_Y

    print "Trying to import raw LIDAR data from %s, %s ..." % (cp.lidar_working_file,cp.gps_working_file)
    errmsg=''
    self.importdir=Blender.sys.dirname(cp.lidar_working_file)
    try:
      lidar_file=open(cp.lidar_working_file,"rb")
    except IOError,(errno,strerror):
      errmsg="IOError #%s: %s" % (errno,strerror)
      print errmsg
      return
    errmsg=''
    self.importdir=Blender.sys.dirname(cp.gps_working_file)
    try:
      gps_file=open(cp.gps_working_file,"rb")
    except IOError,(errno,strerror):
      errmsg="IOError #%s: %s" % (errno,strerror)
      print errmsg
      return
    errmsg=''
    gps_obj=gps_visualization()
    gps_obj.set_file(gps_file)
    gps_obj.skip(cp.startTime)
    pose = gps_obj.currentPose()
    if abs(pose[0]) > MAX_BLENDER_X or abs(pose[1]) > MAX_BLENDER_Y:
      gps_obj.setGPSstartPose()
    gps_obj.dump()
    lidar_obj=lidar_visualization()
    
    if ('symmetric-low-pass-filter' in enableFilters):
      print "enable symmetric low-pass filter"
      lidar_data = LidarData_SLPfiltered(lidar_file,cp.tilt_ignore,cp.startTime,\
                                            cp.endTime,cp.leftAngle+90,cp.rightAngle+90,\
                                             cp.cyclesOfTiltLagRange,cp.windowSize)
    elif ('recursive-low-pass-filter' in enableFilters):
      print "enable recursive low-pass filter"
      lidar_data = LidarData_RLPfiltered(lidar_file,cp.tilt_ignore,cp.startTime,\
                                            cp.endTime,cp.leftAngle+90,cp.rightAngle+90,\
                                             cp.cyclesOfTiltLagRange,cp.rlpf_cutOffFreqInThousandth/1000.0,cp.rlpf_order)
    else:
      lidar_data = LidarData(lidar_file,cp.tilt_ignore,cp.startTime,cp.endTime,cp.cyclesOfTiltLagRange)

    if ('backward-filter' in enableFilters):
      print "enable backward filter"
      lidar_data = BackwardFilter(gps_obj,lidar_data,cp.leftAngle+90,cp.rightAngle+90)
      
    lidar_obj.build_mesh_v1(lidar_data,gps_obj,bDrawVehiclePos)
      
#    lidar_obj.build_mesh(lidar_file,tilt_ignore,gps_obj,bDrawVehiclePos)
    gps_file.close()
    lidar_file.close()
    print "Success"

class gps_import:
  def __init__(self,cp):
    global MAX_BLENDER_X
    global MAX_BLENDER_Y

    print "Trying to import raw GPS data from %s ..." % (cp.gps_working_file)
    try:
      gps_file=open(cp.gps_working_file,"rb")
    except IOError,(errno,strerror):
      errmsg="IOError #%s: %s" % (errno,strerror)
      print errmsg
      return

    self.gps_obj=gps_visualization()
    oldStatus=self.gps_obj.getCheckGPSdataStatus()
    self.gps_obj.setCheckGPSdataStatus(False)

    self.gps_obj.set_file(gps_file)
    self.gps_obj.skip(cp.startTime)
    pose = self.gps_obj.currentPose()
    if abs(pose[0]) > MAX_BLENDER_X or abs(pose[1]) > MAX_BLENDER_Y:
      self.gps_obj.setGPSstartPose()

    self.endTime = cp.endTime
    self.build_mesh()  
    self.gps_obj.setCheckGPSdataStatus(oldStatus)
    gps_file.close()
    print "Success"

  def build_mesh(self):    
    mesh=create_gps_mesh()

    while (self.gps_obj.consume()):
      (time1,time2) = self.gps_obj.getTimes()
      
      if(time2>self.endTime): break
      
      (pose1,pose2) = self.gps_obj.getPoses()
      (mode1,mode2) = self.gps_obj.getModes()
      
      verts = get_gps_verts(pose1,pose2,self.gps_obj.currentError())
      color = getColor(mode1[1])      
      add_gps_to_mesh(mesh, verts,color)

    print "mesh done"
    draw_gps_mesh(mesh)
      
def getColor(gpsPoseType):
  global GPS_POSETYPE_FIXEDPOS,GPS_POSETYPE_FIXEDHEIGHT,GPS_POSETYPE_FIXEDVEL,\
         GPS_POSETYPE_DOPPLER_VELOCITY,GPS_POSETYPE_SINGLE,GPS_POSETYPE_PSRDIFF,\
         GPS_POSETYPE_WAAS,GPS_POSETYPE_OMNISTAR,GPS_POSETYPE_L1_FLOAT,\
         GPS_POSETYPE_IONOFREE_FOAT,GPS_POSETYPE_NARROW_FLOAT,GPS_POSETYPE_L1_INT,\
         GPS_POSETYPE_WIDE_INT,GPS_POSETYPE_NARROW_INT,GPS_POSETYPE_RTK_DIRECT_INS,\
         GPS_POSETYPE_INS,GPS_POSETYPE_OMNISTAR_HP
  
  color=[255,255,255,0]
      
  if (gpsPoseType == GPS_POSETYPE_OMNISTAR_HP):
    color = [0,0,255,0]  # dark blue
  elif (gpsPoseType == GPS_POSETYPE_OMNISTAR):
    color = [255,76,0,0] # red orange
  elif (gpsPoseType == GPS_POSETYPE_WAAS):
    color = [0,51,0,0] # dark green
  elif (gpsPoseType == GPS_POSETYPE_SINGLE):
    color = [0,255,175,0] # light green
  elif (gpsPoseType == GPS_POSETYPE_INS):
    color = [255,76,255,0]  # purple
    
  elif (gpsPoseType == GPS_POSETYPE_RTK_DIRECT_INS):
    color = [255,0,255,0] # rose pink
  elif (gpsPoseType == GPS_POSETYPE_NARROW_INT):
    color = [178,255,255,0] # light blue
  elif (gpsPoseType == GPS_POSETYPE_WIDE_INT):
    color = [0,255,255,0] # blue
  elif (gpsPoseType == GPS_POSETYPE_L1_INT):
    color = [123,76,76,0] # ruby
  elif (gpsPoseType == GPS_POSETYPE_NARROW_FLOAT):
    color = [0,255,0,0] # yellow green
  elif (gpsPoseType == GPS_POSETYPE_IONOFREE_FOAT):
    color = [255,76,76,0] # cerise
  elif (gpsPoseType == GPS_POSETYPE_L1_FLOAT):
    color = [76,76,0,0] # light brown
  elif (gpsPoseType == GPS_POSETYPE_PSRDIFF):
    color = [255,255,76,0] # lemon yellow
  elif (gpsPoseType == GPS_POSETYPE_DOPPLER_VELOCITY):
    color = [76,76,178,0] # violet blue
  elif (gpsPoseType == GPS_POSETYPE_FIXEDVEL):
    color = [255,122,255,0] # lavender
  elif (gpsPoseType == GPS_POSETYPE_FIXEDHEIGHT):
    color = [30,0,66,0] # violet
  elif (gpsPoseType == GPS_POSETYPE_FIXEDPOS):
    color = [0,191,191,0] # blue green

  return color
    
# -----------------------------------------------------------------
# ConfigurableParameters class
# -----------------------------------------------------------------
from ConfigParser import SafeConfigParser
import os.path
import time
class ConfigurableParameters:
  '''
    Current supported configurable parameters are startTime,endTime
    gps_working_file,lidar_working_file, tilt_ignore. Those parameters
    can be changed through UI. Any changes on those parameters will be
    saved in initial file, and will be retrieved when this script startup
  '''
  
  '''
   By default, we set the start time to be the earliest time since 01/01/1970 (epoch)
   end time to be current running time.
  '''
  startTime=0            # time since the epoch,January 1st of 1970, at 0 hours 
  endTime=time.time()    # current running time

  gps_working_file='c:\gps.log'
  lidar_working_file='c:\lidar.log'
  tilt_ignore=20

  leftAngle = -15
  rightAngle = 15
  windowSize  = 7
  bSymmetricLPFilter=0
  bBackwardFilter=0
  bRecursiveLPFilter=0
  rlpf_order=4
  rlpf_cutOffFreqInThousandth=20  # 1/1000
  cyclesOfTiltLagRange = 0
  bGPSerror = 0
  
  _sectionName = "lidar visualization"
  _iniFilename = "lidar.ini"
  
  _START_TIME_optionName = "START_TIME"
  _END_TIME_optionName = "END_TIME"
  _GPS_WORKING_FILE_optionName = "GPS_WORKING_FILE"
  _LIDAR_WORKING_FILE_optionName = "LIDAR_WORKING_FILE"
  _TILT_IGNORE_optionName = "TILT_IGNORE"
  _lidar_file_startTime = 0
  _lidar_file_endTime=time.time()
    
  
  def __init__(self):
    ''' create an instance of ConfigParser'''
    
    self.scp = SafeConfigParser()
    
    '''
       load initial data from configuration file
    '''
    homedir = os.environ.get('HOME')
    if homedir is None:
      homedir = os.environ.get('HOMEPATH')

    if homedir is None:
      homedir = "."    # if either HOME & HOMEPATH is not set, set homedir to local
    else:
      homedir = os.path.join(os.environ.get('HOMEDRIVE'),homedir)  
    
    self.filename = os.path.join(homedir,self._iniFilename)
    self.scp.read(self.filename)
      

    if self.scp.has_section(self._sectionName) == False:
      # first time use, add the section name
      self.scp.add_section(self._sectionName)
      # The configurable pararmeters will be writed to the ini file before
      # this script exits
      return
    
    # read the initial values from ini file
    if self.scp.has_option(self._sectionName,self._START_TIME_optionName):
      timeStr = self.scp.get(self._sectionName,self._START_TIME_optionName)
      try:
        st = time.mktime(time.strptime(timeStr,TIME_STRING_FORMAT))
        self.startTime = st
      except (ValueError,OverflowError):
        pass # do nothing

    if self.scp.has_option(self._sectionName,self._END_TIME_optionName):
      timeStr = self.scp.get(self._sectionName,self._END_TIME_optionName)
      try:
        st = time.mktime(time.strptime(timeStr,TIME_STRING_FORMAT))
        self.endTime = st
      except (ValueError,OverflowError):
        pass # do nothing

    if self.scp.has_option(self._sectionName,self._GPS_WORKING_FILE_optionName):
      self.gps_working_file = self.scp.get(self._sectionName,self._GPS_WORKING_FILE_optionName)

    if self.scp.has_option(self._sectionName,self._LIDAR_WORKING_FILE_optionName):
      self.lidar_working_file = self.scp.get(self._sectionName,self._LIDAR_WORKING_FILE_optionName)
      if self.isValidFilename(self.lidar_working_file):
        # file exists, get the time
        self._lidar_file_startTime,self._lidar_file_endTime = LidarData().getTimeRange(self.lidar_working_file)

    if self.scp.has_option(self._sectionName,self._TILT_IGNORE_optionName):
      self.tilt_ignore = self.scp.getint(self._sectionName,self._TILT_IGNORE_optionName)

    # validate the start & end time
    if self._lidar_file_startTime > 0 and (self.startTime < self._lidar_file_startTime or self.startTime > self._lidar_file_endTime):
      self.startTime = self._lidar_file_startTime

    if self._lidar_file_endTime > 0 and (self.endTime < self._lidar_file_startTime or self.endTime > self._lidar_file_endTime):
      self.endTime = self._lidar_file_endTime
    
  def setLidarWorkingFileTimeRange(self,startTime,endTime):
    self._lidar_file_startTime = startTime
    self._lidar_file_endTime = endTime
    
  def isValidTime(self,time):
    #if self.isValidFilename(self.lidar_working_file):
      #return (self._lidar_file_startTime <= time) and (time <= self._lidar_file_endTime)
    #else:
    return 1

  def isValidFilename(self,filename):
    return os.path.isfile(filename)
    
    
  def _flush(self):
    '''
       only support START_TIME,END_TIME GPS_WORKING_FILE,
       LIDAR_WORKING_FILE and TILT_IGNORE
    '''
    # set the parameters to the SafeConfigParser
    timeStr = time.strftime(TIME_STRING_FORMAT,time.localtime(self.startTime))
    self.scp.set(self._sectionName,self._START_TIME_optionName,timeStr)
    timeStr = time.strftime(TIME_STRING_FORMAT,time.localtime(self.endTime))
    self.scp.set(self._sectionName,self._END_TIME_optionName,timeStr)
    self.scp.set(self._sectionName,self._GPS_WORKING_FILE_optionName,
                                   self.gps_working_file)     
    self.scp.set(self._sectionName,self._LIDAR_WORKING_FILE_optionName,
                                   self.lidar_working_file)     
    self.scp.set(self._sectionName,self._TILT_IGNORE_optionName,
                                   self.tilt_ignore)     
      
    # write the Configuration data to the file 
    try:
      fp = open(self.filename,'w')
    except IOError, (errno, strerror):
      print "***Open file <", self.filename, "> failure, ", strerror
      return
      
    self.scp.write(fp)
    fp.close()
    
  def close(self):
    '''
      Ideally, __del__() is called when this object is deleted. However, it causes
      IOError not defined problem under Blender, I think, because exception classes are
      removed before this destructor is called. Therefor, we need to call close() to
      write parameters to the ini file.
    '''
    self._flush()
    
    
# -----------------------------------------------------------------
# Blender GUI
# -----------------------------------------------------------------

from Blender.BGL import *
from Blender.Draw import *

# each of these corresponds to one of the ui elements
EVENT_TILT_IGNORE=1
EVENT_OPEN_LIDAR=2
EVENT_OPEN_GPS=3
EVENT_LOAD=4
EVENT_EXIT=5
EVENT_STARTTIME=6
EVENT_ENDTIME=7
EVENT_LOAD_VEHICLE_POS=8
EVENT_LEFT_ANGLE=9
EVENT_RIGHT_ANGLE=10
EVENT_WINDOW_SIZE=11
EVENT_SYMMETRIC_LP_FILTER=12
EVENT_BACKWARD_FILTER=13
EVENT_TLR_CYCLES = 14
EVENT_RECURSIVE_LP_FILTER=15
EVENT_RLPF_ORDER=16
EVENT_RLPF_CUTOFFFREQ=17
EVENT_LOAD_VEHICLE_POS_ONLY=18
EVENT_GPS_ERROR=19

def lidar_import_callback(filename):
  '''this is called after selecting something from the file dialog'''
  global CP
  global startTimeTextBox
  
  CP.lidar_working_file = filename
  if CP.isValidFilename(filename):
    startTime,endTime = LidarData().getTimeRange(filename)
  else:
    # get time for GPS file
    startTime,endTime = gps_visualization().getTimeRange(CP.gps_working_file)
    
  if (CP.startTime < startTime or CP.startTime > endTime):
    # out of range,reset it
    CP.startTime = startTime
    startTimeTextBox.val= time.strftime(TIME_STRING_FORMAT,time.localtime(startTime))

  if (CP.endTime < startTime) or (CP.endTime > endTime):
    # out of range,reset it
    CP.endTime = endTime
    endTimeTextBox.val= time.strftime(TIME_STRING_FORMAT,time.localtime(endTime))

  CP.setLidarWorkingFileTimeRange(startTime,endTime)
  
  Blender.Redraw()

def gps_import_callback(filename):
  '''this is called after selecting something from the file dialog'''
  CP.gps_working_file = filename
  if CP.isValidFilename(CP.lidar_working_file) != True:
    # need to update start & end time
    
    startTime,endTime = gps_visualization().getTimeRange(CP.gps_working_file)
    CP.startTime = startTime
    startTimeTextBox.val= time.strftime(TIME_STRING_FORMAT,time.localtime(startTime))
    CP.endTime = endTime
    endTimeTextBox.val= time.strftime(TIME_STRING_FORMAT,time.localtime(endTime))

    Blender.Redraw()

def updateStartTime(timeStr):
  '''
    Converts time string to seconds since the epoch. If the time string is in invalid format,
    the start time is reset to the previous one.
  '''
  global CP
  global startTimeTextBox
  
  errorCode = 0
  try:
    st = time.mktime(time.strptime(timeStr,TIME_STRING_FORMAT))
    if CP.isValidTime(st):
       CP.startTime =  st
    else:
      # the input time is out of range
      errorCode = 1
       
  except (ValueError,OverflowError):
    errorCode = 2

  if (errorCode != 0):
    # to do: popup a message about invalid date time
    startTimeTextBox.val= time.strftime(TIME_STRING_FORMAT,time.localtime(CP.startTime))
    Blender.Redraw()
  
def updateEndTime(timeStr):
  '''
    Converts time string to seconds since the epoch. If the time string is in invalid format,
    the end time is reset to the previous one.
  '''
  global CP
  global endTimeTextBox
  
  errorCode = 0
  try:
    st = time.mktime(time.strptime(timeStr,TIME_STRING_FORMAT))
    if CP.isValidTime(st):
       CP.endTime = st
    else:
      # the input time is out of range
      errorCode = 1
       
  except (ValueError,OverflowError):
    errorCode = 2
    
  if (errorCode != 0):
    # to do: popup a message about invalid date time
    endTimeTextBox.val= time.strftime(TIME_STRING_FORMAT,time.localtime(CP.endTime))
    Blender.Redraw()
    
def draw_event():
  '''blender calls this to draw the scripts window, put Blender.Draw and Blender.BGL stuff here'''
  global EVENT_TILT_IGNORE,EVENT_OPEN_LIDAR,EVENT_OPEN_GPS,EVENT_LOAD,EVENT_EXIT,\
         EVENT_STARTTIME,EVENT_ENDTIME,EVENT_LOAD_VEHICLE_POS,EVENT_LEFT_ANGLE,\
         EVENT_RIGHT_ANGLE,EVENT_WINDOW_SIZE,EVENT_SYMMETRIC_LP_FILTER,EVENT_BACKWARD_FILTER,\
         EVENT_TLR_CYCLES,EVENT_RECURSIVE_LP_FILTER,EVENT_RLPF_ORDER,EVENT_RLPF_CUTOFFFREQ,\
         EVENT_LOAD_VEHICLE_POS_ONLY, EVENT_GPS_ERROR

  global CP
  global tilt_ignore_button
  global startTimeTextBox
  global endTimeTextBox
  global windowSizeButton
  global leftAngleButton
  global rightAngleButton
  global symmetricLPFilterToggle
  global backwardFilterToggle
  global cyclesOfTiltLagRangeButton
  global recursiveLPFilterToggle
  global rlpf_orderButton
  global rlpf_cutoffFreqButton
  

  # some opengl stuff to draw text
  glClear(GL_COLOR_BUFFER_BIT)
  glRasterPos2d(8,403)
  Text("Overbot LIDAR Visualization")
  glRasterPos2d(8,373)
  Text("Current Working File:")
  PushButton("LIDAR",EVENT_OPEN_LIDAR,8,347,60,18,"Select a lidar file and set it as the current working file")
  glRasterPos2d(80,353)
  Text(CP.lidar_working_file)
  PushButton("GPS",EVENT_OPEN_GPS,8,317,60,18,"Select a gps file and set it as the current working file")
  glRasterPos2d(80,323)
  Text(CP.gps_working_file)

  # the ui elements
  startTimeStr = time.strftime(TIME_STRING_FORMAT,time.localtime(CP.startTime))
  startTimeTextBox = String("Start time: ",EVENT_STARTTIME,8,283,210,20,
                              startTimeStr,40)
  endTimeStr = time.strftime(TIME_STRING_FORMAT,time.localtime(CP.endTime))
  endTimeTextBox = String("End  time: ",EVENT_ENDTIME,8,251,210,20,
                              endTimeStr,40)

  cyclesOfTiltLagRangeButton=Number("Cycles Of Tilt Lag Range",EVENT_TLR_CYCLES,8,213,210,18,CP.cyclesOfTiltLagRange,-50,50,"Set the number of cycles that tilt lags range. Press Reload after changing this value!")
  rlpf_orderButton=Number("RLPF Order",EVENT_RLPF_ORDER,8,173,175,18,CP.rlpf_order,0,20,"Set butterworth filter order number when loading. Press Reload after changing this value!")
  rlpf_cutoffFreqButton=Number("RLPF Cutoff Freq ",EVENT_RLPF_CUTOFFFREQ,195,173,175,18,CP.rlpf_cutOffFreqInThousandth,0,50,"Set butterworth cut off frequency in thousandth. Press Reload after changing this value!")
  tilt_ignore_button=Number("Tilt Ignore",EVENT_TILT_IGNORE,8,133,175,18,CP.tilt_ignore,0,90,"Set the number of degrees of tilt ignored when loading. Press Reload after changing this value!")
  windowSizeButton=Number("SLPF Window Size",EVENT_WINDOW_SIZE,195,133,175,18,CP.windowSize,3,91,"Set the low-pass filter window size. Press Reload after changing this value!")
  leftAngleButton=Number("Left Angle",EVENT_LEFT_ANGLE,8,93,175,18,CP.leftAngle,-90,0,"Set the number of degrees left from straight ahead. Press Reload after changing this value!")
  rightAngleButton=Number("Right Angle",EVENT_RIGHT_ANGLE,195,93,175,18,CP.rightAngle,0,90,"Set the number of degrees right from straight ahead. Press Reload after changing this value!")
  symmetricLPFilterToggle=Toggle("symmetric LP Filter",EVENT_SYMMETRIC_LP_FILTER,8,53,110,18,CP.bSymmetricLPFilter,"Enable/disable symmetric Low-Pass filter. Press Reload after changing this value!")
  backwardFilterToggle=Toggle("backward Filter",EVENT_BACKWARD_FILTER,130,53,110,18,CP.bBackwardFilter,"Enable/disable backward filter. Press Reload after changing this value!")
  recursiveLPFilterToggle=Toggle("Recursive LP Filter",EVENT_RECURSIVE_LP_FILTER,252,53,120,18,CP.bRecursiveLPFilter,"Enable/disable recursive Low-Pass filter. Press Reload after changing this value!")
  GPSerrorToggle=Toggle("Show GPS Error",EVENT_GPS_ERROR,380,53,120,18,CP.bGPSerror,"Show GPS uncertainty with vehicle position. Press Reload after changing this value!")
  PushButton("Load",EVENT_LOAD,8,10,50,18,"Load the current working file")
  PushButton("Load With Vehicle Pos",EVENT_LOAD_VEHICLE_POS,65,10,140,18,"Draw vechicle positions& scan lines")
  PushButton("Load Vehicle Pos Only",EVENT_LOAD_VEHICLE_POS_ONLY,210,10,140,18,"Draw vechicle positions only")
  PushButton("Exit",EVENT_EXIT,355,10,50,18,"Terminate the script (ESC)")

def event(ev,val):
  '''catch Blender.Draw keyboard events here'''
  if(ev==ESCKEY and val==0):
    CP.close()
    Exit()

def button_event(ev):
  '''catch ui element events here'''
  global EVENT_TILT_IGNORE,EVENT_OPEN_LIDAR,EVENT_OPEN_GPS,EVENT_LOAD,EVENT_EXIT,\
         EVENT_STARTTIME,EVENT_ENDTIME,EVENT_LOAD_VEHICLE_POS,EVENT_LEFT_ANGLE,\
         EVENT_RIGHT_ANGLE,EVENT_WINDOW_SIZE,EVENT_SYMMETRIC_LP_FILTER,EVENT_BACKWARD_FILTER,\
         EVENT_TLR_CYCLES,EVENT_RECURSIVE_LP_FILTER,EVENT_RLPF_ORDER,EVENT_RLPF_CUTOFFFREQ,\
         EVENT_LOAD_VEHICLE_POS_ONLY, EVENT_GPS_ERROR
  global opendialog_file
  global CP
  global tilt_ignore_button
  global leftAngleButton
  global rightAngleButton
  global windowSizeButton
  global symmetricLPFilterToggle
  global backwardFilterToggle
  global cyclesOfTiltLagRangeButton
  global recursiveLPFilterToggle
  global rlpf_orderButton
  global rlpf_cutoffFreqButton

  if(ev==EVENT_TILT_IGNORE):
    CP.tilt_ignore = tilt_ignore_button.val
    
  if(ev==EVENT_OPEN_LIDAR):
    Blender.Window.FileSelector(lidar_import_callback,"Import Overbot LIDAR","*.log")
    
  if(ev==EVENT_OPEN_GPS):
    Blender.Window.FileSelector(gps_import_callback,"Import Overbot GPS","*.gpslog")
    
  if(ev==EVENT_LOAD):
    enableFilters=[]
    if CP.bSymmetricLPFilter:
      enableFilters.append('symmetric-low-pass-filter')
    if CP.bBackwardFilter:
      enableFilters.append('backward-filter')
    if CP.bRecursiveLPFilter:
      enableFilters.append('recursive-low-pass-filter')
      
    lidar=lidar_import(CP,enableFilters)
    Blender.Redraw()
    
  if(ev==EVENT_LOAD_VEHICLE_POS):
    enableFilters=[]
    if CP.bSymmetricLPFilter:
      enableFilters.append('symmetric-low-pass-filter')
    if CP.bBackwardFilter:
      enableFilters.append('backward-filter')
    if CP.bRecursiveLPFilter:
      enableFilters.append('recursive-low-pass-filter')
      
    lidar=lidar_import(CP,enableFilters,True)
    Blender.Redraw()

  if(ev==EVENT_LOAD_VEHICLE_POS_ONLY):
    lidar=gps_import(CP)
    Blender.Redraw()
    
  if (ev == EVENT_STARTTIME):
      updateStartTime(startTimeTextBox.val)
      
  if (ev == EVENT_ENDTIME):
      updateEndTime(endTimeTextBox.val)
      
  if(ev==EVENT_LEFT_ANGLE):
    CP.leftAngle = leftAngleButton.val
    
  if(ev==EVENT_RIGHT_ANGLE):
    CP.rightAngle = rightAngleButton.val
    
  if(ev==EVENT_WINDOW_SIZE):
    # make sure to be odd number
    if (windowSizeButton.val > CP.windowSize):
      CP.windowSize = windowSizeButton.val+1
    elif (windowSizeButton.val < CP.windowSize):
      CP.windowSize = windowSizeButton.val-1
      
    windowSizeButton.val = CP.windowSize
    Redraw()
    
  if(ev==EVENT_SYMMETRIC_LP_FILTER):
    CP.bSymmetricLPFilter = 1 - CP.bSymmetricLPFilter
    
    if CP.bRecursiveLPFilter and CP.bSymmetricLPFilter:
      # recursive low-pass filter & symmetric low-pass filter cannot be enabled at the same time
      CP.bRecursiveLPFilter = 0
    Redraw()
    
  if(ev==EVENT_RECURSIVE_LP_FILTER):
    CP.bRecursiveLPFilter = 1 - CP.bRecursiveLPFilter
    
    if CP.bRecursiveLPFilter and CP.bSymmetricLPFilter:
      # butterworth filter & symmetric low-pass filter cannot be enabled at the same time
      CP.bSymmetricLPFilter = 0
      
    Redraw()
    
  if(ev==EVENT_BACKWARD_FILTER):
    CP.bBackwardFilter = 1 - CP.bBackwardFilter
    Redraw()
  
  if(ev==EVENT_GPS_ERROR):
    CP.bGPSerror = 1 - CP.bGPSerror
    Redraw()
  
  if(ev==EVENT_TLR_CYCLES):
    CP.cyclesOfTiltLagRange = cyclesOfTiltLagRangeButton.val
    
  if(ev==EVENT_RLPF_ORDER):
    CP.rlpf_order = rlpf_orderButton.val
    print CP.rlpf_order

  if(ev==EVENT_RLPF_CUTOFFFREQ):
    CP.rlpf_cutOffFreqInThousandth = rlpf_cutoffFreqButton.val
    
  if(ev==EVENT_EXIT):
    CP.close()
    Exit()

#-------------------------------------------------------------------------------
#  MAIN
#-------------------------------------------------------------------------------
global VEHICLE_MOVING_DIR

if VEHICLE_MOVING_DIR:
    print "Yaw is equal to the angle of vehicle moving direction"
    
# initial the ConfigurableParameters
CP = ConfigurableParameters()
'''
tilt_ignore_button=Create(CP.tilt_ignore)
leftAngleButton=Create(CP.leftAngle)
rightAngleButton=Create(CP.rightAngle)
windowSizeButton=Create(CP.windowSize)
cyclesOfTiltLagRangeButton = Create(CP.cyclesOfTiltLagRange)
rlpf_orderButton=Create(CP.rlpf_order)
rlpf_cutoffFreqButton=Create(CP.rlpf_cutOffFreqInThousandth)
'''

# this begins the blender ui event loop
# probably not a good idea to put code after here
Register(draw_event,event,button_event)
