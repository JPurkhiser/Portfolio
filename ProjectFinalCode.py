from abaqus import *
from abaqusConstants import *
import __main__
import section
import odbSection
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import job
import sketch
import visualization
import xyPlot
import connectorBehavior
import displayGroupOdbToolset as dgo
from math import atan, sin, cos, tan
from Post_P_Script import getResults
from Post_P_Script import findEigenValue
from Post_P_Script import getDisplacement

session.journalOptions.setValues(replayGeometry=COORDINATE,recoverGeometry=COORDINATE)

######################################
# Variable and Fixed Design Parameters
######################################

##########################
# FEA Modeling Parameters
# (e.g., mesh seeds, step times, etc)
##########################
DataFile = open('PostDataNew.txt','w')
DataFile.write('ArmT, ArmD, LegT, BaseT, HoleL, mat, stress, dmax, ev, mass\n')
DataFile.close()
####################################
### Calculated Properties/Values ###
####################################

ArmT = 0.922 #0.25 to 1
ArmD = 15.062 #10 to 15
LegT = 35 #25 to 35
BaseT = 20 #20 to 30
HoleL = 50 #20 to 80
mat=1

# ArmT = 1
# ArmD = 14.19
# LegT = 25
# BaseT = 30
# HoleL = 33.21
# mat=1

BaseT = BaseT/2.0

# [steel,ti] #New material added Steel Al 7075 yeild=73000psi  Ti-8Al-1Mo yeild=132000psi
E=[10400000,17400000] # lbf/in^2
rho=[0.102,0.158] # lbm/in^3
nu=[0.33,0.32] # [ ]
load_cond=1

Mdb() 
  
# Sketch Geometry and Create Parts
print 'Sketching/Creating the part'
#Extrusion
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=1000.0)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)
s.Spot(point=(0.0, 50.0))
s.Spot(point=(0.0, -50.0))
s.Spot(point=(-385.0, BaseT))
s.Spot(point=(-385.0, -BaseT))
s.Line(point1=(-385.0, BaseT), point2=(0.0, 50.0))
s.Line(point1=(-385.0, -BaseT), point2=(0.0, -50.0))
s.Spot(point=(0.0, 40.0))
s.Spot(point=(0.0, -40.0))
s.Arc3Points(point1=(-385.0, -BaseT), point2=(-385.0, BaseT), point3=(-385.0-BaseT, 
    0.0))
s.Line(point1=(0.0, 50.0), point2=(0.0, 40.0))
s.VerticalConstraint(entity=g.findAt((0.0, 45.0)), addUndoState=False)
s.Line(point1=(0.0, -50.0), point2=(0.0, -40.0))
s.VerticalConstraint(entity=g.findAt((0.0, -45.0)), addUndoState=False)
#
s.Spot(point=(-60.0, 34.23)) #34.23 is (top pt on slant at -60 - 10)
s.Spot(point=(-60.0, -34.23))
s.Spot(point=(-94.23, 0.0))
s.ArcByCenterEnds(center=(-60.0, 0.0), point1=(-60.0, -34.23), point2=(-60.0, 
    34.23), direction=CLOCKWISE)
s.Line(point1=(-60.0, 34.23), point2=(0.0, 40.0))
s.Line(point1=(-60.0, -34.23), point2=(0.0, -40.0))
#

p = mdb.models['Model-1'].Part(name='Part-1', dimensionality=THREE_D, 
    type=DEFORMABLE_BODY)
p = mdb.models['Model-1'].parts['Part-1']
p.BaseSolidExtrude(sketch=s, depth=70.0)
s.unsetPrimaryObject()
p = mdb.models['Model-1'].parts['Part-1']



#Extrude
p = mdb.models['Model-1'].parts['Part-1']
f, e = p.faces, p.edges
t = p.MakeSketchTransform(sketchPlane=f.findAt(coordinates=(0.0, 46.666667, 
    33.333333)), sketchUpEdge=e.findAt(coordinates=(0.0, 47.5, 0.0)), 
    sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 25.0))
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=141.42, gridSpacing=3.53, transform=t)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-1']
p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)


s.Line(point1=(15.0, 70.0), point2=(0.0, 0.0)) #15 is legT
s.Line(point1=(0.0, 0.0), point2=(15.0, -70.0))
s.Line(point1=(15.0, -70.0), point2=(35.0, -70.0)) #25 is depth/2, 50 is depth
s.Line(point1=(35.0, -70.0), point2=(35.0, 70.0))
s.Line(point1=(35.0, 70.0), point2=(15.0, 70.0))
s.Line(point1=(-LegT, 0.0), point2=(15-LegT, 70.0))
s.Line(point1=(15-LegT, 70.0), point2=(-70.0, 70.0))
s.Line(point1=(-70.0, 70.0), point2=(-70.0, -70.0))
s.Line(point1=(-70.0, -70.0), point2=(15-LegT, -70.0))
s.Line(point1=(15-LegT, -70.0), point2=(-LegT, 0.0))
p = mdb.models['Model-1'].parts['Part-1']
f1, e1 = p.faces, p.edges
p.CutExtrude(sketchPlane=f1.findAt(coordinates=(0.0, 46.666667, 33.333333)), 
    sketchUpEdge=e1.findAt(coordinates=(0.0, 47.5, 0.0)), 
    sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s, 
    flipExtrudeDirection=OFF)
s.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']



#Extrude for Arm cavity
p = mdb.models['Model-1'].parts['Part-1']
f, e = p.faces, p.edges
t = p.MakeSketchTransform(sketchPlane=f.findAt(coordinates=(0.0, 43.333333, 
    22.0)), sketchUpEdge=e.findAt(coordinates=(0.0, 50.0, 15)), 
    sketchPlaneSide=SIDE1, sketchOrientation=TOP, origin=(0.0, 0.0, 25.0))
s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=141.42, gridSpacing=3.53, transform=t)
g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
s1.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-1']
p.projectReferencesOntoSketch(sketch=s1, filter=COPLANAR_EDGES)

s1.Line(point1=(-ArmD/2.0, ArmD/2.0), point2=(-ArmD/2.0, -ArmD/2.0))

s1.Line(point1=(-ArmD/2.0, -ArmD/2.0), point2=(20.0, -ArmD/2.0))

s1.Line(point1=(20.0, -ArmD/2.0), point2=(20.0, ArmD/2.0))

s1.Line(point1=(20.0, ArmD/2.0), point2=(-ArmD/2.0, ArmD/2.0))

p = mdb.models['Model-1'].parts['Part-1']
f1, e1 = p.faces, p.edges
p.CutExtrude(sketchPlane=f1.findAt(coordinates=(0.0, 43.333333, 22.0)), 
    sketchUpEdge=e1.findAt(coordinates=(0.0, 50.0, 15)), 
    sketchPlaneSide=SIDE1, sketchOrientation=TOP, sketch=s1, depth=365.0,
    flipExtrudeDirection=OFF)
s1.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']


#BASE CUT NEW
p = mdb.models['Model-1'].parts['Part-1']
p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=0.0)

p = mdb.models['Model-1'].parts['Part-1']
e, d = p.edges, p.datums
t = p.MakeSketchTransform(sketchPlane=d[4], sketchUpEdge=e.findAt(coordinates=(
    -373.25, 0.0, 25.0)), sketchPlaneSide=SIDE1, sketchOrientation=TOP, 
    origin=(-(385.0+BaseT), 0.0, 25 + LegT))
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=900.5, gridSpacing=22.51, transform=t)
g, v, d1, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-1']
p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)

s.Line(point1=(0.0, LegT/2.0), point2=(LegT/2.00000*2.7474, 0.0)) #7.5 is half of tl and 20.606 is tl/2*tan(70) to get 20 deg cut
s.Line(point1=(LegT/2.0*2.7474, 0.0), point2=(0.0, 0.0))
s.Line(point1=(0.0, 0.0), point2=(0.0, LegT/2.0))
p = mdb.models['Model-1'].parts['Part-1']
e1, d2 = p.edges, p.datums

p.CutExtrude(sketchPlane=d2[4], sketchUpEdge=e1.findAt(coordinates=(-373.25, 
    0.0, 25.0)), sketchPlaneSide=SIDE1, sketchOrientation=TOP, sketch=s, 
    flipExtrudeDirection=ON)
s.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']

p = mdb.models['Model-1'].parts['Part-1'] #Same steps again for extrusion on other side of datum plane
e, d = p.edges, p.datums
t = p.MakeSketchTransform(sketchPlane=d[4], sketchUpEdge=e.findAt(coordinates=(
    -373.25, 0.0, 25.0)), sketchPlaneSide=SIDE1, sketchOrientation=TOP, 
    origin=(-(385.0+BaseT), 0.0, 25 + LegT))
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=900.5, gridSpacing=22.51, transform=t)
g, v, d1, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-1']
p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)

s.Line(point1=(0.0, LegT/2.0), point2=(LegT/2.0*2.7474, 0.0)) #7.5 is half of tl and 20.606 is tl/2*tan(70 to get 20 deg cut
s.Line(point1=(LegT/2.0*2.7474, 0.0), point2=(0.0, 0.0))
s.Line(point1=(0.0, 0.0), point2=(0.0, LegT/2.0))
p = mdb.models['Model-1'].parts['Part-1']
e1, d2 = p.edges, p.datums
p.CutExtrude(sketchPlane=d2[4], sketchUpEdge=e1.findAt(coordinates=(-373.25, 
    0.0, 25.0)), sketchPlaneSide=SIDE1, sketchOrientation=TOP, sketch=s, 
    flipExtrudeDirection=OFF)
s.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']


#Topology Cut
p = mdb.models['Model-1'].parts['Part-1']
p.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=0.0)

p = mdb.models['Model-1'].parts['Part-1']
e1, d2 = p.edges, p.datums
t = p.MakeSketchTransform(sketchPlane=d2[7], sketchUpEdge=e1.findAt(
    coordinates=(-365.0, 2.5, 25+ArmD/2.0)), sketchPlaneSide=SIDE1, 
    sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0))
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=905.02, gridSpacing=22.62, transform=t)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-1']
p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
session.viewports['Viewport: 1'].view.setValues(session.views['Back'])

s.Line(point1=(120.0, 15.0), point2=(120.0, 25.0))
s.Line(point1=(120.0, 15.0), point2=((120+HoleL), 15.0))
s.Line(point1=((120+HoleL), 15.0), point2=((120+HoleL), 20.0))
s.Line(point1=((120+HoleL), 20.0), point2=(120.0, 25.0))


s.Line(point1=(120.0, -15.0), point2=(120.0, -25.0))
s.Line(point1=(120.0, -15.0), point2=((120+HoleL), -15.0))
s.Line(point1=((120+HoleL), -15.0), point2=((120+HoleL), -20.0))
s.Line(point1=((120+HoleL), -20.0), point2=(120.0, -25.0))


p = mdb.models['Model-1'].parts['Part-1']
e1, d2 = p.edges, p.datums
p.CutExtrude(sketchPlane=d2[7], sketchUpEdge=e1.findAt(coordinates=(-365.0, 
    2.5, 25+ArmD/2.0)), sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s, 
    flipExtrudeDirection=ON)
s.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']


#Leg Partition
p = mdb.models['Model-1'].parts['Part-1']
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=-60.0)
p = mdb.models['Model-1'].parts['Part-1']
c = p.cells
pickedCells = c.findAt(((0.0, 43.333333, 39.047618), ))
d = p.datums
p.PartitionCellByDatumPlane(datumPlane=d[9], cells=pickedCells)

p = mdb.models['Model-1'].parts['Part-1']
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=-95.0)
p = mdb.models['Model-1'].parts['Part-1']
c = p.cells
pickedCells = c.findAt(((-146.666667, -15.0, 45.119047), ))
d1 = p.datums
p.PartitionCellByDatumPlane(datumPlane=d1[11], cells=pickedCells)

p = mdb.models['Model-1'].parts['Part-1']
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=-120.0)
p = mdb.models['Model-1'].parts['Part-1']
c = p.cells
pickedCells = c.findAt(((-173.333333, -21.666667, 43.690476), ))
d = p.datums
p.PartitionCellByDatumPlane(datumPlane=d[13], cells=pickedCells)

p = mdb.models['Model-1'].parts['Part-1']
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=-(120 + HoleL))
p = mdb.models['Model-1'].parts['Part-1']
c = p.cells
pickedCells = c.findAt(((-173.333333, -21.666667, 43.690476), ))
d1 = p.datums
p.PartitionCellByDatumPlane(datumPlane=d1[15], cells=pickedCells)

p = mdb.models['Model-1'].parts['Part-1']
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=-345.0)
p = mdb.models['Model-1'].parts['Part-1']
c = p.cells
pickedCells = c.findAt(((-200.0, -16.666667, 44.761905), ))
d = p.datums
p.PartitionCellByDatumPlane(datumPlane=d[17], cells=pickedCells)


#Creating Arm 1
print "Creating Arm 1"
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=200.0)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)
s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(ArmD/2.0, 0.0))
s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=((ArmD)/2.0 - ArmT, 0.0))
p = mdb.models['Model-1'].Part(name='Arm1', dimensionality=THREE_D, 
    type=DEFORMABLE_BODY)
p = mdb.models['Model-1'].parts['Arm1']
p.BaseSolidExtrude(sketch=s, depth=157.84)
s.unsetPrimaryObject()
p = mdb.models['Model-1'].parts['Arm1']
del mdb.models['Model-1'].sketches['__profile__']

##Creating Arm 2"
print "Creating Arm 2"
s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=200.0)
g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
s1.setPrimaryObject(option=STANDALONE)
s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=((ArmD)/2.0 - ArmT, 0.0))
#s1.delete(objectList=(g.findAt((-13.75, 0.0)), ))
s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=((ArmD)/2.0 - 2.0*ArmT, 0.0))
p = mdb.models['Model-1'].Part(name='Arm2', dimensionality=THREE_D, 
    type=DEFORMABLE_BODY)
p = mdb.models['Model-1'].parts['Arm2']
p.BaseSolidExtrude(sketch=s1, depth=157.84)
s1.unsetPrimaryObject()
p = mdb.models['Model-1'].parts['Arm2']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
del mdb.models['Model-1'].sketches['__profile__']

##Creating Arm 3"
print "Creating Arm 3"
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=200.0)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)
s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=((ArmD)/2.0 - 2.0*ArmT, 0.0))
s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=((ArmD)/2.0 - 3.0*ArmT, 0.0))
p = mdb.models['Model-1'].Part(name='Arm3', dimensionality=THREE_D, 
    type=DEFORMABLE_BODY)
p = mdb.models['Model-1'].parts['Arm3']
p.BaseSolidExtrude(sketch=s, depth=157.84)
s.unsetPrimaryObject()
p = mdb.models['Model-1'].parts['Arm3']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
del mdb.models['Model-1'].sketches['__profile__']
##
p = mdb.models['Model-1'].parts['Arm3']
v1, e1, d1, n1 = p.vertices, p.edges, p.datums, p.nodes
p.ReferencePoint(point=p.InterestingPoint(edge=e1[3], rule=CENTER))

#Defining the face partitions
print 'Partitioning part'


# Create Material
print 'Creating the Materials'
#creates Carbon Fiber Material (E in GPa)
if mat<=0.5:
    mat=0
else:
    mat=1
mdb.models['Model-1'].Material(name='Material-1')
mdb.models['Model-1'].materials['Material-1'].Elastic(table=((E[int(mat)], nu[int(mat)]), 
    ))
mdb.models['Model-1'].materials['Material-1'].Density(table=((rho[int(mat)], ), ))

#Create/Assign Section
print('Creating the Sections')
print('Assigning the Sections')
mdb.models['Model-1'].HomogeneousSolidSection(name='Section-1', 
    material='Material-1', thickness=None)

	
#Create/Assign Section
print 'Creating/Assigning the Sections'
#Assigning to Arm 3
p = mdb.models['Model-1'].parts['Arm3']
c = p.cells
cells = c.findAt((((ArmD)/2.0 - 3.0*ArmT, 0, 0), ))
region = p.Set(cells=cells, name='Set-1')
p = mdb.models['Model-1'].parts['Arm3']
p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)

#Assigning to Arm 2
p = mdb.models['Model-1'].parts['Arm2']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
p = mdb.models['Model-1'].parts['Arm2']
c = p.cells
cells = c.findAt((((ArmD)/2.0 - 2.0*ArmT, 0, 5), ))
region = p.Set(cells=cells, name='Set-1')
p = mdb.models['Model-1'].parts['Arm2']
p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)

# #Assigning to Arm 1
p = mdb.models['Model-1'].parts['Arm1']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
p = mdb.models['Model-1'].parts['Arm1']
c = p.cells
cells = c.findAt((((ArmD)/2.0 - ArmT, 0, 5), ))
region = p.Set(cells=cells, name='Set-1')
p = mdb.models['Model-1'].parts['Arm1']
p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)

# #Assigning to Leg
p = mdb.models['Model-1'].parts['Part-1']
c = p.cells
cells = c.findAt(((-365.0, -4, 28.0), ), ((-133.0, 25.0, 40), ), ((-126.666667, 0.0, 41.385715), ),
 ((-300, 0.0, 39.868181), ), ((-110, 30.0, 36.300325), ), ((
    -83.333333, -35, 35.687663), ), ((-40.0, -41, 34.750649), ), 
    ((-133.0, -30.0, 40), ), ((-40.0, 41, 34.750649), 
    ))
region = p.Set(cells=cells, name='Set-3')
p = mdb.models['Model-1'].parts['Part-1']
p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)

#Assemble Parts
print 'Placing Parts in Space'
a = mdb.models['Model-1'].rootAssembly
a.DatumCsysByDefault(CARTESIAN)
p = mdb.models['Model-1'].parts['Arm1']
a.Instance(name='Arm1-1', part=p, dependent=ON)
p = mdb.models['Model-1'].parts['Arm2']
a.Instance(name='Arm2-1', part=p, dependent=ON)
p = mdb.models['Model-1'].parts['Arm3']
a.Instance(name='Arm3-1', part=p, dependent=ON)

 #Translate Arm 1
a = mdb.models['Model-1'].rootAssembly
a.translate(instanceList=('Arm1-1', ), vector=(0.0, 0.0, -156.59))
#: The instance Arm1-1 was translated by 0., 0., -156.59 with respect to the assembly coordinate system

#Translate Arm 2
a = mdb.models['Model-1'].rootAssembly
a.translate(instanceList=('Arm2-1', ), vector=(0.0, 0.0, -310.43))
#: The instance Arm2-1 was translated by 0., 0., -310.43 with respect to the assembly coordinate system

#Translation Arm 3
a = mdb.models['Model-1'].rootAssembly
a.translate(instanceList=('Arm3-1', ), vector=(0.0, 0.0, -464.27))
#: The instance Arm3-1 was translated by 0., 0., -464.27 with respect to the assembly coordinate system,

#Ties Arm 2 & Arm 1 Together
a = mdb.models['Model-1'].rootAssembly
s1 = a.instances['Arm1-1'].faces
side1Faces1 = s1.findAt(((ArmD/2.0 - ArmT, 0.0, -155.59), )) #Face of Arm 1 Cylinder that is facing Arm 2
region1=a.Surface(side1Faces=side1Faces1, name='s_Surf-1')
a = mdb.models['Model-1'].rootAssembly
s1 = a.instances['Arm2-1'].faces
side1Faces1 = s1.findAt(((ArmD/2.0 - ArmT, 0.0, -155.59), )) #Face of Arm 2 Cylinder that is facing Arm 2
region2=a.Surface(side1Faces=side1Faces1, name='s_Surf-2')
mdb.models['Model-1'].Tie(name='Constraint-2', master=region1, slave=region2, 
    positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)

#Ties Arm 3 & Arm 2 Together
a = mdb.models['Model-1'].rootAssembly
s1 = a.instances['Arm2-1'].faces
side1Faces1 = s1.findAt(((ArmD/2.0 - 2.0*ArmT, 0.0, -309.43), )) #Face of Arm 2 Cylinder that is facing Arm 3
region1=a.Surface(side1Faces=side1Faces1, name='s_Surf-3')
a = mdb.models['Model-1'].rootAssembly
s1 = a.instances['Arm3-1'].faces
side1Faces1 = s1.findAt(((ArmD/2.0 - 2.0*ArmT, 0.0, -309.43), )) #Face of Arm 3 Cylinder that is facing Arm 3
region2=a.Surface(side1Faces=side1Faces1, name='s_Surf-4')
mdb.models['Model-1'].Tie(name='Constraint-1', master=region1, slave=region2, 
    positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)
   
#Arm Partitions
#Arm1
p = mdb.models['Model-1'].parts['Arm1']
p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=0.0)
p = mdb.models['Model-1'].parts['Arm1']
c = p.cells
pickedCells = c.findAt(((ArmD/2.0 - ArmT, 0.0, 0.0), ))
d1 = p.datums
p.PartitionCellByDatumPlane(datumPlane=d1[3], cells=pickedCells)

#Arm2
p = mdb.models['Model-1'].parts['Arm2']
p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=0.0)
p = mdb.models['Model-1'].parts['Arm2']
c = p.cells
pickedCells = c.findAt(((ArmD/2.0 - 2.0*ArmT, 0.0, 0.0), ))
d = p.datums
p.PartitionCellByDatumPlane(datumPlane=d[3], cells=pickedCells)

#Arm3
p = mdb.models['Model-1'].parts['Arm3']
p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=0.0)
p = mdb.models['Model-1'].parts['Arm3']
c = p.cells
pickedCells = c.findAt(((ArmD/2.0 - 3.0*ArmT, 0.0, 0.0), ))
d1 = p.datums
p.PartitionCellByDatumPlane(datumPlane=d1[4], cells=pickedCells)

a = mdb.models['Model-1'].rootAssembly
p = mdb.models['Model-1'].parts['Part-1']
a.Instance(name='Part-1-1', part=p, dependent=ON)

a = mdb.models['Model-1'].rootAssembly
a.rotate(instanceList=('Arm1-1', 'Arm2-1', 'Arm3-1'), axisPoint=(-(ArmD - 2*ArmT)/2, 0.0, 
    -156.59), axisDirection=(13.25, 0.0, 0.0), angle=180.0)
#: The instances were rotated by 180. degrees about the axis defined by the point -7.25, 0., -156.59 and the vector 13.25, 0., 0.

a = mdb.models['Model-1'].rootAssembly
a.translate(instanceList=('Arm1-1', 'Arm2-1', 'Arm3-1'), vector=(-359.0, 0.0, 
    -121.09))
#: The instances were translated by -359., 0., -121.09 with respect to the assembly coordinate system

a = mdb.models['Model-1'].rootAssembly
a.rotate(instanceList=('Arm1-1', 'Arm2-1', 'Arm3-1'), axisPoint=(-359.0, 6.0, 
    30.0), axisDirection=(0.0, -12.0, 0.0), angle=65.0)
#: The instances were rotated by 65. degrees about the axis defined by the point -359., 6., 30. and the vector 0., -12., 0.

#Create Instances here
a = mdb.models['Model-1'].rootAssembly
a.translate(instanceList=('Part-1-1', ), vector=(6.0, 0.0, 2.4625))

#Create Interaction here

#Ref point on leg
p = mdb.models['Model-1'].parts['Part-1']
# p.ReferencePoint(point=(-365.0, 0.0, 25+ArmD/2.0000 - 2.4625))
p.ReferencePoint(point=(-365,0,30))


print 'Creating Interactions'

a = mdb.models['Model-1'].rootAssembly
a.regenerate()

#Tie Connection - James
a = mdb.models['Model-1'].rootAssembly
s1 = a.instances['Part-1-1'].faces
side1Faces1 = s1.findAt(((-359.0, -5.0, 30.0), ))
region1=a.Surface(side1Faces=side1Faces1, name='m_Surf-5')
a = mdb.models['Model-1'].rootAssembly
s1 = a.instances['Arm3-1'].faces
side1Faces1 = s1.findAt(((-359, -(ArmD/2.0 -2.0*ArmT - 0.05), 30), ), ((-359, (ArmD/2.0 -2.0*ArmT - 0.05), 30), ))
region2=a.Surface(side1Faces=side1Faces1, name='s_Surf-5')
mdb.models['Model-1'].Tie(name='Constraint-3', master=region1, slave=region2, 
    positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=OFF, 
    thickness=ON)
            

#Define Steps
print 'Defining the Steps'
#Static Step
mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial')

# #Create Loads
print 'Defining Loads'
if load_cond==0:
    e1 = a.instances['Part-1-1'].edges
    a = mdb.models['Model-1'].rootAssembly
    s1 = a.instances['Part-1-1'].faces
    side1Faces1 = s1.findAt((((-(385.0+BaseT) + 6) + LegT/4.0000*2.7474, 0.0,((25 + LegT) + 2.4625) - LegT/4.0000), ))
    region = a.Surface(side1Faces=side1Faces1, name='Surf-5')
    mdb.models['Model-1'].SurfaceTraction(name='Load-1', createStepName='Step-1', 
        region=region, magnitude=13.0, directionVector=((0.0, 0.0, 0.0), 
        (1.0, 0.0, -2.7474)), distributionType=UNIFORM, field='', localCsys=None, 
        traction=GENERAL)
else:
    e1 = a.instances['Part-1-1'].edges
    a = mdb.models['Model-1'].rootAssembly
    s1 = a.instances['Part-1-1'].faces
    side1Faces1 = s1.findAt((((-(385.0+BaseT) + 6) + LegT/4.0000*2.7474, 0.0,((25 + LegT) + 2.4625) - LegT/4.0000), ))
    region = a.Surface(side1Faces=side1Faces1, name='Surf-5')
    mdb.models['Model-1'].SurfaceTraction(name='Load-1', createStepName='Step-1', 
        region=region, magnitude=13.0, directionVector=((0.0, 0.0, 0.0), 
        (1.0, 0.0, -1.7320)), distributionType=UNIFORM, field='', localCsys=None, 
        traction=GENERAL)


# #Define BCs
print 'Defining all BCs'
a = mdb.models['Model-1'].rootAssembly
f1 = a.instances['Part-1-1'].faces
faces1 = f1.findAt(((6.0, -45.0, 35), ), ((6.0, 45.0, 35), ))
region = a.Set(faces=faces1, name='Set-1')
mdb.models['Model-1'].PinnedBC(name='BC-1', createStepName='Initial', 
    region=region, localCsys=None)
    
#Arm Bc
a = mdb.models['Model-1'].rootAssembly
f1 = a.instances['Arm1-1'].faces
faces1 = f1.findAt(((62.904401,ArmD/2.0 - ArmT/1.5,-166.737253), ), 
((62.904401,-(ArmD/2.0 - ArmT/2.0),-166.737253), ))
region = a.Set(faces=faces1, name='Set-3')
mdb.models['Model-1'].PinnedBC(name='BC-2', createStepName='Initial', 
    region=region, localCsys=None)
    
    
#Cell Set
a = mdb.models['Model-1'].rootAssembly
c1 = a.instances['Arm1-1'].cells
cells1 = c1.getByBoundingBox(-800,-800,-800,800,800,800)
c2 = a.instances['Arm2-1'].cells
cells2 = c2.getByBoundingBox(-800,-800,-800,800,800,800)
c3 = a.instances['Arm3-1'].cells
cells3 = c3.getByBoundingBox(-800,-800,-800,800,800,800)
# r3 = a.instances['Arm3-1'].referencePoints
# refPoints3=(r3[2], )
c4 = a.instances['Part-1-1'].cells
cells4 = c4.getByBoundingBox(-800,-800,-800,800,800,800)
# r4 = a.instances['Part-1-1'].referencePoints
# refPoints4=(r4[22], )
a.Set(cells=cells1+cells2+cells3+cells4, name='ALL_PART')

#LegMesh
p = mdb.models['Model-1'].parts['Part-1']
p.seedPart(size=7, deviationFactor=0.1, minSizeFactor=0.1)

#Setting Bad regions to Pink
p = mdb.models['Model-1'].parts['Part-1']
c = p.cells
pickedRegions = c.findAt(((-365.0, -5.333333, 27.095238), ), ((-83.333333, 
    -35.0, 39.474335), ))
p.setMeshControls(regions=pickedRegions, elemShape=TET, technique=FREE)
elemType1 = mesh.ElemType(elemCode=C3D20R)
elemType2 = mesh.ElemType(elemCode=C3D15)
elemType3 = mesh.ElemType(elemCode=C3D10)
p = mdb.models['Model-1'].parts['Part-1']
c = p.cells
cells = c.findAt(((-365.0, -5.333333, 27.095238), ), ((-83.333333, -35.0, 
    39.474335), ))
pickedRegions =(cells, )
p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
    elemType3))

p = mdb.models['Model-1'].parts['Part-1']
p.generateMesh()

#Arm1 Mesh
p = mdb.models['Model-1'].parts['Arm1']
p.seedPart(size=2.5, deviationFactor=0.1, minSizeFactor=0.1)
p = mdb.models['Model-1'].parts['Arm1']
p.generateMesh()

#Arm2 Mesh
p = mdb.models['Model-1'].parts['Arm2']
p.seedPart(size=2.5, deviationFactor=0.1, minSizeFactor=0.1)
p = mdb.models['Model-1'].parts['Arm2']
p.generateMesh()

#Arm3 Mesh
p = mdb.models['Model-1'].parts['Arm3']
p.seedPart(size=2.5, deviationFactor=0.1, minSizeFactor=0.1)
p = mdb.models['Model-1'].parts['Arm3']
p.generateMesh()

# Job for Stress 
ModelName='Model-1'
StepName='Step-1'
mdb.Job(name=ModelName, model=ModelName, description='', type=ANALYSIS, 
    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
    scratch='', multiprocessingMode=DEFAULT, numCpus=1, numGPUs=0)

job=mdb.jobs[ModelName]

# delete lock file, which for some reason tends to hang around, if it exists
if os.access('%s.lck'%ModelName,os.F_OK):
    os.remove('%s.lck'%ModelName)
    
# Run the job, then process the results.        
job.submit()
job.waitForCompletion()
print('Completed Stress job')

#Extracting Results
#Stress 
stress=getResults(ModelName)

#Deflection
dmax=getDisplacement(ModelName)

#Mass
prop=mdb.models[ModelName].rootAssembly.getMassProperties()
mass=prop['mass']

#Performing Job again for Buckling

#Deleting Stress Step
del mdb.models['Model-1'].steps['Step-1']

#Creating New Buckling Step
a = mdb.models['Model-1'].rootAssembly
mdb.models['Model-1'].BuckleStep(name='Step-1', previous='Initial', numEigen=1, 
    eigensolver=LANCZOS, minEigen=None, blockSize=DEFAULT, maxBlocks=DEFAULT)

#Applying Loads Again
print 'Defining Loads'
if load_cond==0:
    e1 = a.instances['Part-1-1'].edges
    a = mdb.models['Model-1'].rootAssembly
    s1 = a.instances['Part-1-1'].faces
    side1Faces1 = s1.findAt((((-(385.0+BaseT) + 6) + LegT/4.0000*2.7474, 0.0,((25 + LegT) + 2.4625) - LegT/4.0000), ))
    region = a.Surface(side1Faces=side1Faces1, name='Surf-5')
    mdb.models['Model-1'].SurfaceTraction(name='Load-1', createStepName='Step-1', 
        region=region, magnitude=13.0, directionVector=((0.0, 0.0, 0.0), 
        (1.0, 0.0, -2.7474)), distributionType=UNIFORM, field='', localCsys=None, 
        traction=GENERAL)
else:
    e1 = a.instances['Part-1-1'].edges
    a = mdb.models['Model-1'].rootAssembly
    s1 = a.instances['Part-1-1'].faces
    side1Faces1 = s1.findAt((((-(385.0+BaseT) + 6) + LegT/4.0000*2.7474, 0.0,((25 + LegT) + 2.4625) - LegT/4.0000), ))
    region = a.Surface(side1Faces=side1Faces1, name='Surf-5')
    mdb.models['Model-1'].SurfaceTraction(name='Load-1', createStepName='Step-1', 
        region=region, magnitude=13.0, directionVector=((0.0, 0.0, 0.0), 
        (1.0, 0.0, -1.7320)), distributionType=UNIFORM, field='', localCsys=None, 
        traction=GENERAL)
        
#Job for Buckling
ModelName='Model-1'
StepName='Step-1'
mdb.Job(name=ModelName, model=ModelName, description='', type=ANALYSIS, 
    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
    scratch='', multiprocessingMode=DEFAULT, numCpus=1, numGPUs=0)

job=mdb.jobs[ModelName]

# delete lock file, which for some reason tends to hang around, if it exists
if os.access('%s.lck'%ModelName,os.F_OK):
    os.remove('%s.lck'%ModelName)
    
# Run the job, then process the results.        
job.submit()
job.waitForCompletion()
print('Completed Buckling job')

#Extracting Results       
#Eigen Value
ev = findEigenValue(ModelName,StepName)

DataFile = open('PostDataNew.txt','a')
DataFile.write('%10f %10f %10f %10f %10f %10f %10f %10f %10f %10f\n' % (ArmT, ArmD, LegT, BaseT*2, HoleL, mat, stress, dmax, ev, mass))
DataFile.close()	


print 'DONE!!'
