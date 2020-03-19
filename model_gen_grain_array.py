import __main__
import part
import mesh
import section
import regionToolset
from abaqus import *
from abaqusConstants import *
import __main__
import re
import os
import numpy as np
from inspect import getsourcefile
# Twin hardness model creation
# Parametric variable: twin angle

# Parameters:



def random_texture(n):    
    U = np.random.uniform(-1.0, 1.0, n)
    V = np.random.uniform(0.0, 2.0 * np.pi, n)
    W = np.random.uniform(0.0, 2.0 * np.pi, n)
    return U, V, W

backwardCompatibility.setValues(includeDeprecated = True, reportDeprecated = False)

def create_shell(w, h, model_name='Model-1', part_name='Part-1'):
    model  = mdb.models[model_name]
    model.Part(dimensionality=THREE_D, name=part_name, type=DEFORMABLE_BODY)
    part   = mdb.models[model_name].parts[part_name]
    sketch = model.Sketch(name='base', sheetSize=w)
    model.sketches['base'].rectangle(point1=(0.000, 0.000),point2=(w, h))
    part.BaseShell(sketch=sketch)
    # del mdb.models[model_name].sketches['__profile__']

    return model, part

def extrude_shell(model, part, part_name = "part-1", extrudeDepth=5, N=1):
    # Offset all faces
    bottomFaces = part.faces[:]
    part.OffsetFaces(faceList=bottomFaces, distance=extrudeDepth)

    # Get the location of bottom faces
    bottomFacesLoc = [face.pointOn[0] for face in bottomFaces][:]

    for i, faceLoc in enumerate(bottomFacesLoc[:]):
        # Create material name
        mat_name = 'g%d' % (N+i)
        print("Extruding: " + mat_name)

        # Select the top and bottom faces using previously obtained locations
        faceBottom = part.faces.findAt((faceLoc[0], faceLoc[1], faceLoc[2]))
        faceTop = part.faces.findAt((faceLoc[0], faceLoc[1], faceLoc[2] - extrudeDepth))

        # Get edges from the selection
        bottomLoop = [part.edges[j] for j in faceBottom.getEdges()]
        topLoop = [part.edges[j] for j in faceTop.getEdges()]

        # Blend faces
        part.BlendFaces(side1=bottomLoop, side2=topLoop, method=SHORTEST_PATH)
        faceBottom = part.faces.findAt((faceLoc[0], faceLoc[1], faceLoc[2]))
        faceTop = part.faces.findAt((faceLoc[0], faceLoc[1], faceLoc[2] - extrudeDepth))

        # Create cell from top and bottom face
        part.AddCells(faceList=[faceBottom, faceTop])

    return N + i + 1

def hex_array(w, h, r, model, part):
    k = int(w/(2*r))+1
    x = float(w)/2
    y = float(h)/2
    
    p = mdb.models['Model-1'].parts['Part-1']
    f, e, d = p.faces, p.edges, p.datums
    t = p.MakeSketchTransform(sketchPlane=f.findAt(coordinates=(x, 
        y, 0.0), normal=(0.0, 0.0, 1.0)), sketchUpEdge=e.findAt(
        coordinates=(w, h/4, 0.0)), sketchPlaneSide=SIDE1, origin=(0, 0, 
        0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=113.13, gridSpacing=2.82, transform=t)

    l = (np.cos(np.radians(30))*r)
    
    width = [x+(2*l*i) for i in range(-k,k+1,1)]

    horz = [y]*len(width)
    width = zip(width,horz)

    # The hex patterning algorithm climbs diagonally and populates horizontally and vertically per 
    # diagonal hex
    for x1,y1 in width:
        xs = [x1+(i*l) for i in range(-k,k+1,1)]
        ys = [y1+(i*r*1.5) for i in range(-k,k+1,1)]
        coords = zip(xs,ys)
        for x_coord,y_coord in coords:
            s = draw_hexagon(x_coord,y_coord,r,s)
    p = mdb.models['Model-1'].parts['Part-1']
    f = p.faces
    pickedFaces = f.findAt(((x, y, 0.0), ))
    e1, d2 = p.edges, p.datums
    p.PartitionFaceBySketch(sketchUpEdge=e1.findAt(coordinates=(w, h/4, 0.0)),
        faces=pickedFaces, sketch=s)
    p = mdb.models['Model-1'].parts['Part-1']
    n = 1
    for i, cell in enumerate(p.faces):
        n += 1
    return n

def draw_hexagon(x,y,r,s):
    theta = np.radians(60)
    sinr = np.sin(theta)*r
    cosr = np.cos(theta)*r
    sinr2 = np.sin(theta/2)*r
    cosr2 = np.cos(theta/2)*r
    x1 = x
    y1 = y
    s.Line(point1=(x1, y1+r), point2=(sinr + x1, cosr + y1)) #
    s.Line(point1=(sinr + x1, cosr + y1), point2=(cosr2 + x1, -sinr2 + y1))
    s.Line(point1=(cosr2 + x1, -sinr2 + y1), point2=(x1, y1-r))
    s.Line(point1=(x1, y1-r), point2=(-cosr2 + x1, -sinr2 + y1))
    s.Line(point1=(-cosr2 + x1, -sinr2 + y1), point2=(-sinr + x1, cosr + y1))
    s.Line(point1=(-sinr + x1, cosr + y1), point2=(x1, y1+r))
    return s

def draw_twin(x, y, r, t, angle):
    l = np.cos(np.radians(30))*r
    p = mdb.models['Model-1'].parts['Part-1']
    f = p.faces
    e1 = p.edges
    
    t = p.MakeSketchTransform(sketchPlane=f.findAt(coordinates=(x, y, 
        0.0), normal=(0.0, 0.0, 1.0)), sketchUpEdge=e1.findAt(coordinates=(
        (x+l), y, 0.0)), sketchPlaneSide=SIDE1, origin=(x, y, 0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=113.13, gridSpacing=2.82, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    p = mdb.models['Model-1'].parts['Part-1']
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.Line(point1=(-l, 0.0), point2=(l, 0.0))
    s.HorizontalConstraint(entity=g.findAt((0.0, 0.0)), addUndoState=False)
    s.setAsConstruction(objectList=(g.findAt((0.0, 0.0)), ))
    s.Line(point1=(-1.41, -23.97), point2=(-1.41, 23.2649999999488))
    s.Line(point1=(1.41, 23.265), point2=(1.41, -23.97))
    s.Spot(point=(0.0, 0.0))
    s.CoincidentConstraint(entity1=v.findAt((0.0, 0.0)), entity2=g.findAt((0.0, 
        0.0)), addUndoState=False)
    s.FixedConstraint(entity=g.findAt((0.0, 0.0)))
    s.ParallelConstraint(entity1=g.findAt((-1.41, -0.3525)), entity2=g.findAt((
        1.41, -0.3525)))
    s.DistanceDimension(entity1=g.findAt((-1.41, -0.3525)), entity2=g.findAt((1.41, 
        -0.3525)), textPoint=(0.131233215332031, 0.866680145263672), value=0.5)
    s.DistanceDimension(entity1=g.findAt((1.41, -0.3525)), entity2=v.findAt((0.0, 
        0.0)), textPoint=(0.577840805053711, -0.634544372558594), value=0.25)
    s.AngularDimension(line1=g.findAt((0.25, -0.3525)), line2=g.findAt((0.0, 0.0)), 
        textPoint=(0.98594856262207, 0.381668090820313), value=angle)
    p = mdb.models['Model-1'].parts['Part-1']
    f = p.faces
    pickedFaces = f.findAt(((x, x+l, 0.0), ))
    e, d1 = p.edges, p.datums
    p.PartitionFaceBySketch(sketchUpEdge=e.findAt(coordinates=(x*2, (y*2)/2, 
        0.0)), faces=pickedFaces, sketch=s)
    s.unsetPrimaryObject()
    del mdb.models['Model-1'].sketches['__profile__']


def vecs2rotm(vector1, vector2):
	# Finds rotation matrix for rotating on vector onto another
	
	v = np.cross(vector1,vector2)/np.linalg.norm(np.cross(vector1, vector2))
	theta = np.arccos((np.dot(vector1,vector2))/(np.dot(np.linalg.norm(vector1),np.linalg.norm(vector2))))
	mat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
	# Get rotation matrix
	if (vector1==vector2).all():
		rotm = np.eye(3)
	else:
		rotm = np.eye(3,3) + (np.sin(theta) * mat) + ((1-np.cos(theta)) * mat.dot(mat))
	return rotm

def axang2rotm(axis,theta):
    v = np.transpose(axis)
    theta = float(theta)
    theta = np.radians(theta)
    mat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    # Get rotation matrix
    if theta == 0:
        rotm = np.eye(3)
    else:
        rotm = np.eye(3,3) + (mat * np.sin(theta)) + (np.dot(mat,mat) * (1-np.cos(theta)))
    return  rotm

def create_orientations(angle = 45, twist = 0):
	# Creates rotation matrices for twin and parent for
    # given LD and TB angle measured from x to TB in counter-clockwise
    # manner. Note that twin and parent namesakes here are meaningless.
    # twist is applied on normal plane in clockwise direction
    import numpy as np
    # angle must be between 0 - 90 
    angle = np.radians(angle)

    n = np.array([0.57735027,0.57735027,0.57735027])
    t = np.array(np.dot(1/(np.sqrt(6)/2),[-(1/2), -(1/2), 1]))
    s1 = np.array([0.70710678,-0.70710678,0.00000000])

    # Morphological directions of twin boundary
    twin_n = np.array([-np.sin(angle), np.cos(angle), 0])
    twin_t = np.array([np.cos(angle), np.sin(angle), 0])

    # First rotate normal to desired vector parent

    rotm_p = vecs2rotm(n, twin_n) # reference to twin
    sc = np.dot(rotm_p, s1) # current s1 vector
    rotm_temp = vecs2rotm(sc, twin_t) #align s1 with twin boundary
    rotm_p = np.dot(rotm_temp, rotm_p)  # updated rotm
    nb = np.dot(rotm_p,n) # normal from twin boundary
    rotm_temp = axang2rotm(nb, twist) # apply twist around said normal
    rotm_p = np.dot(rotm_temp, rotm_p) # updated

    # twin - repeated procedure for above but anti-parallel and twisted 60 degrees to conform to twin crystallography

    rotm_t = vecs2rotm(n, -twin_n)
    sc = np.dot(rotm_t, s1)
    rotm_temp = vecs2rotm(sc, twin_t)
    rotm_t = np.dot(rotm_temp, rotm_t)
    nb = np.dot(rotm_t, n)
    rotm_temp = axang2rotm(nb, 60-twist)
    rotm_t = np.dot(rotm_temp,rotm_t)

    return rotm_p, rotm_t

def unpack(rotm):
	import numpy as np
	result = []
	for i in range(0,3):
		for j in range(0,3):
			result.append(rotm[i][j])
	return result

def generate_materials(U,V,W,part, grains, r, twin_w, twin_length, tb_angle, x, y,twist, extrude_depth = 5.0):
    p = part
    c = p.cells
    ta = (twin_w*twin_length*extrude_depth)
    hexa = ((3*np.sqrt(3))/2)*(r**2)*extrude_depth

    centre = np.array((x, y, -extrude_depth/2))

    #
    Theta = np.pi - np.arccos(U)
    Phi = V
    Psi = W
    #
    # Rotation matrix
    Elements = [None] * 9
    #
    Elements[0] = lambda phi, theta, psi: -np.sin(phi) * np.sin(psi) * np.cos(theta) + np.cos(phi) * np.cos(psi)
    Elements[1] = lambda phi, theta, psi: np.sin(phi) * np.cos(psi) + np.sin(psi) * np.cos(phi) * np.cos(theta)
    Elements[2] = lambda phi, theta, psi: np.sin(psi) * np.sin(theta)
    Elements[3] = lambda phi, theta, psi: -np.sin(phi) * np.cos(psi) * np.cos(theta) - np.sin(psi) * np.cos(phi)
    Elements[4] = lambda phi, theta, psi: -np.sin(phi) * np.sin(psi) + np.cos(phi) * np.cos(psi) * np.cos(theta)
    Elements[5] = lambda phi, theta, psi: np.sin(theta) * np.cos(psi)
    Elements[6] = lambda phi, theta, psi: np.sin(phi) * np.sin(theta)
    Elements[7] = lambda phi, theta, psi: -np.sin(theta) * np.cos(phi)
    Elements[8] = lambda phi, theta, psi: np.cos(theta)
    
    n=0

    # Assign cells, twins based on size and location
    for i, cell in enumerate(c):
        a = c[i].getSize()
        location = c[i].pointOn
        cell_loc = np.array((location[0][0] ,location[0][1], location[0][2]))
        dist = np.linalg.norm(centre - cell_loc)
        crit = np.sqrt(r**2+extrude_depth**2)
        print("a", centre, "b", cell_loc, "dist:", dist)
        if a <= ta and dist < crit:
            generate_TB((c[i],), twin=True, angle=tb_angle,twist=twist)
        elif a > ta and a < hexa/2 and dist < crit:
            generate_TB((c[i],), twin=False, angle=tb_angle,twist=twist)
        else:
            grain="g{}".format(str(n))
            mdb.models['Model-1'].Material(name=grain)
            mdb.models['Model-1'].materials[grain].Depvar(n=106)
            props = (2.0, ) + tuple([Elements[j](Phi[n], Theta[n], Psi[n]) for j in range(9)])
            mdb.models['Model-1'].materials[grain].UserMaterial(mechanicalConstants=props)
            mdb.models['Model-1'].HomogeneousSolidSection(name=grain, material=grain, 
                thickness=None)
            region = (c[i],)
            p.SectionAssignment(region=region, sectionName=grain, offset=0.0, 
                offsetType=MIDDLE_SURFACE, offsetField='', 
                thicknessAssignment=FROM_SECTION)
            n+=1

def generate_TB(cells, twin = False, angle=45,twist=0, model_name = 'Model-1', part_name = 'Part-1'):
    p = mdb.models['Model-1'].parts['Part-1']
    # is twin
    if twin == True:
        parent, twin = create_orientations(angle,twist)
        mdb.models[model_name].Material(name='twin')
        mdb.models[model_name].materials['twin'].Depvar(n=106)
        props = (2.0, ) + tuple(unpack(twin))
        mdb.models[model_name].materials['twin'].UserMaterial(mechanicalConstants=props)
        mdb.models['Model-1'].HomogeneousSolidSection(name='twin', material='twin', 
        thickness=None)
        region = cells
        p.SectionAssignment(region=region, sectionName='twin', offset=0.0, 
            offsetType=MIDDLE_SURFACE, offsetField='', 
            thicknessAssignment=FROM_SECTION)
    # is parent
    elif twin == False:
        parent, twin = create_orientations(angle)
        mdb.models[model_name].Material(name='parent')
        mdb.models[model_name].materials['parent'].Depvar(n=106)
        props = (2.0, ) + tuple(unpack(parent))
        mdb.models[model_name].materials['parent'].UserMaterial(mechanicalConstants=props)
        # Create sections:
        mdb.models['Model-1'].HomogeneousSolidSection(name='parent', material='parent', 
            thickness=None)
        region = cells
        p.SectionAssignment(region=region, sectionName='parent', offset=0.0, 
            offsetType=MIDDLE_SURFACE, offsetField='', 
            thicknessAssignment=FROM_SECTION)
    

def assemble_structure():
    a = mdb.models['Model-1'].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models['Model-1'].parts['Part-1']
    a.Instance(name='Part-1-1', part=p, dependent=ON)


def create_step():
    a = mdb.models['Model-1'].rootAssembly
    mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial', 
        timePeriod=11.0, maxNumInc=100000, initialInc=0.01, minInc=1e-10, 
        maxInc=0.1, nlgeom=ON)
    mdb.models['Model-1'].fieldOutputRequests['F-Output-1'].setValues(variables=(
        'S', 'E', 'U', 'RF', 'SDV'), frequency=10)

def face_selector(assembly, w, l, depth):
    a = mdb.models['Model-1'].rootAssembly
    f = a.instances['Part-1-1'].faces

    left = []
    right = []
    top = []
    bottom = []
    back = []
    front = []
    for i, face in enumerate(f):
        x, y ,z =f[i].getCentroid()[0]
        if x == 0 and y > 0 and z == -depth/2:
            # left
            left.append(f[i].pointOn)
        elif x > 0 and y == 0 and z == -depth/2:
            # bottom
            bottom.append(f[i].pointOn)
        elif x == w and y > 0 and z == -depth/2:
            # right
            right.append(f[i].pointOn)
        elif x > 0 and y > 0 and z == -depth:
            # back
            back.append(f[i].pointOn)
        elif x > 0 and y > 0 and z == 0:
            # Front
            front.append(f[i].pointOn)
        elif y == w and x > 0 and z == -depth/2:
            # top
            top.append(f[i].pointOn)
    
    left = tuple(left)
    right = tuple(right)
    top = tuple(top)
    bottom = tuple(bottom)
    back = tuple(back)
    front = tuple(front)
    return left, top, right, bottom, back, front

def create_bcs(strain, w, depth = 5.0):
    a = mdb.models['Model-1'].rootAssembly
    f = a.instances['Part-1-1'].faces
    left, top, right, bottom, back, front = face_selector(a, w, w, depth)
    # Load
    right = tuple([f.findAt(i) for i in right])
    region = a.Set(faces=right, name='RIGHT')
    mdb.models['Model-1'].DisplacementBC(name='RIGHT', createStepName='Step-1', 
        region=region, u1=(w*strain), u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
        localCsys=None)
    # Bottom BC
    bottom = tuple([f.findAt(i) for i in bottom])
    region = a.Set(faces=bottom, name='BOTTOM')
    mdb.models['Model-1'].DisplacementBC(name='BOTTOM', createStepName='Step-1', 
        region=region, u1=UNSET, u2=0.0, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
        localCsys=None)
    # Left constraint
    print(left)
    left = tuple([f.findAt(i) for i in left])
    region = a.Set(faces=left, name='LEFT')
    mdb.models['Model-1'].DisplacementBC(name='LEFT', createStepName='Step-1', 
        region=region, u1=0.0, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
        localCsys=None)
    #bottom
    back = tuple([f.findAt(i) for i in back])    
    region = a.Set(faces=back, name='BACK')
    mdb.models['Model-1'].DisplacementBC(name='BACK', createStepName='Step-1', 
        region=region, u1=UNSET, u2=UNSET, u3=0.0, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
        amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
        localCsys=None)

    
def mesh_part(x,y,seed_size=0.5):
    x = float(x)
    y = float(y)
    p = mdb.models['Model-1'].parts['Part-1']
    region = []
    for cell in p.cells:
        region.append(cell)
    p.setMeshControls(regions=region, technique=SWEEP)
    p.seedPart(size=seed_size, deviationFactor=0.1, minSizeFactor=0.1)
    elemType1 = mesh.ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
    elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
    p.setElementType(regions=region, elemTypes=(elemType1, elemType2, 
        elemType3))
    # refine twin and parent
    e = p.edges
    twin_face = p.faces.findAt((x,y, 0),)
    edges = twin_face.getEdges()
    edges = tuple(e[i] for i in edges)
    p.seedEdgeBySize(edges=edges, size=0.1, deviationFactor=0.1, 
    minSizeFactor=0.1, constraint=FINER)
    p.generateMesh()


def gen_eqn_const(w,h,z=5.0):

    a = mdb.models['Model-1'].parts['Part-1']
    n = a.nodes
    master = n.getClosest((w, h, 0), )
    master_label = master.label
    slaves = n.getByBoundingBox(xMin=0.0,yMin=w,zMin=-z,xMax=w,yMax=w,zMax=0.0)
    slaves_list = [i.label for i in slaves if i.coordinates != master.coordinates]
    slaves = n.sequenceFromLabels(tuple(slaves_list),)
    a.Set(nodes = slaves, name = "slaves")
    a.SetFromNodeLabels(nodeLabels=((master_label),), name = "master")

    mdb.models['Model-1'].Equation(name='Constraint-1', terms=((1.0, 
        'Part-1-1.slaves', 2), (-1.0, 'Part-1-1.master', 2)))


def gen_inp(inp_name):
    mdb.Job(name=inp_name, model='Model-1', description='', type=ANALYSIS, 
    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
    scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=1, 
    numGPUs=0)
    mdb.jobs[inp_name].writeInput(consistencyChecking=OFF)

def gen_pic(job_name, model_name, part_name, dir_path):
    m = mdb.models[model_name]
    p = m.parts[part_name]

    
    file_id = dir_path + "/" + job_name
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    session.viewports['Viewport: 1'].view.setValues(cameraPosition=(7.9214, 
        7.28779, 42.8217), cameraUpVector=(0, 1, 0))
    session.viewports['Viewport: 1'].enableMultipleColors()
    session.viewports['Viewport: 1'].setColor(initialColor='#BDBDBD')
    cmap=session.viewports['Viewport: 1'].colorMappings['Material']
    session.viewports['Viewport: 1'].setColor(colorMapping=cmap)
    session.viewports['Viewport: 1'].disableMultipleColors()
    session.tiffOptions.setValues(imageSize=(3000, 1254))
    session.printToFile(
        fileName=str(job_name) + "meshquality" + '.tif', 
        format=TIFF, canvasObjects=(session.viewports['Viewport: 1'], ))


def run_gen(w,h,r,twin_angle,U,V,W, dir_path,twist):
    # Run function which goes through commands
    x = float(w)/2
    y = float(h)/2
    t = 0.25
    element_size = 0.5
    strain = 0.06

    model, part = create_shell(w, h)
    n = hex_array(w, h, r, model, part)
    draw_twin(x,y,r,t,twin_angle)
    model = mdb.models['Model-1']
    part = mdb.models['Model-1'].parts['Part-1']
    extrude_shell(model,part)
    generate_materials(U,V,W,part, n, r, 0.5, r*2, twin_angle,x,y,twist)
    assemble_structure()
    create_step()
    create_bcs(strain, w)
    mesh_part(x,y,element_size)
    gen_eqn_const(w,h,z=5.0)
    d = str(r*2)
    number = d[0] +  "-" + d[2]  + "um"
    twin_angle = int(twin_angle)
    name = "inclination_{}".format(str(twin_angle))
    gen_inp(name)
    gen_pic(name, "Model-1", 'Part-1',dir_path)

def schmid_calc(rotm, loading_direction):
    # returns the schmid of all system
    pass
    return schmid_dictionary

# w, h = 15.0, 15.0
# r = 2.5
# U,V,W = random_texture(1000)
# twin_angle_range = list(range(0,95,5))

# dir_path = "D:\Modelling\TwinHardness\Angle_dependance_individual_slip"

# grain_size = list(range(10,50,5))
# grain_size = [float(i)/10 for i in grain_size]

# for radius in grain_size:
#     Mdb()
#     print(radius)
#     run_gen(w, h, radius, 45, U, V, W, dir_path)
    
# for angle in twin_angle_range:
#     Mdb()
#     run_gen(w, h, r, angle, U,V,W, dir_path)

# rotate crystal

dir_path = "D:\Modelling\Calibration models\TwinHardness\Crystal_twist"
w, h = 15.0, 15.0
r = 2.5
U,V,W = random_texture(1000)
inclinations = [float(i) for i in range(0,60,5)]


for i in inclinations:
    Mdb()
    run_gen(w, h, r, i, U, V, W, dir_path, 0)

