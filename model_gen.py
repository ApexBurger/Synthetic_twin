#Import stack

from abaqus import *
from abaqusConstants import *
import numpy as np




# Parameters

part_name = 'Part-1'
model_name = 'Model-1'
model_width = 200
model_height = 200
extrude_depth = 5.0
twin = [45, -90, 0]
width = 100
height = 100
r = 0.5 #radius of inclusion

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
	part = model.parts[part_name]
    # Offset all faces
	bottomFaces = part.faces[:]
	part.OffsetFaces(faceList=bottomFaces, distance=extrudeDepth)

	# Get the location of bottom faces
	bottomFacesLoc = [face.pointOn[0] for face in bottomFaces][:]

	for i, faceLoc in enumerate(bottomFacesLoc[:]):
		# Create material name
		mat_name = 'g%d' % (N+i)
		print ("Extruding: " + mat_name)

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

		# Assign material
		model.Material(name=mat_name)
		model.materials[mat_name].UserMaterial(mechanicalConstants=(0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))
		model.materials[mat_name].Depvar(n=92)
		model.HomogeneousSolidSection(material=mat_name, name=mat_name)
		region = (part.cells[0],)
		part.SectionAssignment(region=region, sectionName=mat_name)

	return N + i + 1

def assign_random_texture(model = 'Model-1', phase=2):
    #
    # Get list of the materials
    materials = mdb.models[model].materials.keys()
    n = len(materials)
    #
    U = np.random.uniform(-1.0, 1.0, n)
    V = np.random.uniform(0.0, 2.0 * np.pi, n)
    W = np.random.uniform(0.0, 2.0 * np.pi, n)
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
    #
    count = 0
    for i, material in enumerate(materials):
        count += 1
        args = (phase, ) + tuple(Elements[j](Phi[i], Theta[i], Psi[i]) for j in range(9))
        rotm = np.reshape(args[1:], (3,3))
        if np.linalg.det(rotm) < 0.999  and np.linalg.det(rotm) > 1.001 :
            print ("Something wrong with rotation matrix:", count,  np.linalg.det(rotm), rotm)
            break
        mdb.models[model].materials[material].UserMaterial(mechanicalConstants=args)

    print "random textures assigned" 

def partition_shell():
	f, e, d1 = p.faces, p.edges, p.datums
	t = p.MakeSketchTransform(sketchPlane=f.findAt(coordinates=(33.333333, 
	    33.333333, 0.0), normal=(0.0, 0.0, 1.0)), sketchUpEdge=e.findAt(
	    coordinates=(100.0, 25.0, 0.0)), sketchPlaneSide=SIDE1, origin=(50.0, 50.0, 
	    0.0))
	s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
	    sheetSize=282.84, gridSpacing=7.07, transform=t)
	g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
	s1.setPrimaryObject(option=SUPERIMPOSE)
	p = mdb.models['Model-1'].parts['Part-1']
	p.projectReferencesOntoSketch(sketch=s1, filter=COPLANAR_EDGES)

def gen_eqn_const(w,h,z=5.0,model_name="Model-1",part_name="Part-1"):
	m = mdb.models[model_name]
	a = m.parts[part_name]
	n = a.nodes
	master = n.getClosest((w, h, 0), )
	master_label = master.label
	slaves = n.getByBoundingBox(xMin=0.0,yMin=w,zMin=-z,xMax=w,yMax=w,zMax=0.0)
	slaves_list = [i.label for i in slaves if i.coordinates != master.coordinates]
	slaves = n.sequenceFromLabels(tuple(slaves_list),)
	a.Set(nodes = slaves, name = "slaves")
	a.SetFromNodeLabels(nodeLabels=((master_label),), name = "master")

	m.Equation(name='Constraint-1', terms=((1.0, 
		'{}-1.slaves'.format(part_name), 2), (-1.0, '{}-1.master'.format(part_name), 2)))

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

def create_bcs(strain, w, depth = 5.0, model_name = "model-1", part_name = "part-1"):
	m = mdb.models[model_name]
	a = m.rootAssembly
	f = a.instances[part_name].faces

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
	m.DisplacementBC(name='BOTTOM', createStepName='Step-1', 
	region=region, u1=UNSET, u2=0.0, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
	amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
	localCsys=None)
	# Left constraint
	print(left)
	left = tuple([f.findAt(i) for i in left])
	region = a.Set(faces=left, name='LEFT')
	m.DisplacementBC(name='LEFT', createStepName='Step-1', 
	region=region, u1=0.0, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
	amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
	localCsys=None)
	#bottom
	back = tuple([f.findAt(i) for i in back])    
	region = a.Set(faces=back, name='BACK')
	m.DisplacementBC(name='BACK', createStepName='Step-1', 
	region=region, u1=UNSET, u2=UNSET, u3=0.0, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
	amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
	localCsys=None)

def generate_TB(angle=45,twist=0, depvar=105, csys = 2, model_name = 'Model-1'):
	m = mdb.models[model_name]

	parent, twin = create_orientations(angle,twist)
	print("TWIN:", twin, "PARENT:", parent)
	m.Material(name='twin')
	m.materials['twin'].Depvar(n=depvar)
	props = (csys, ) + tuple(unpack(twin))
	m.materials['twin'].UserMaterial(mechanicalConstants=props)
	m.HomogeneousSolidSection(name='twin', material='twin', 
		thickness=None)

	m.Material(name='parent')
	m.materials['parent'].Depvar(n=depvar)
	props = (csys, ) + tuple(unpack(parent))
	m.materials['parent'].UserMaterial(mechanicalConstants=props)
	# Create sections:
	m.HomogeneousSolidSection(name='parent', material='parent', 
		thickness=None)

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

