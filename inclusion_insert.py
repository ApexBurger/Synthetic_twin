# Mdb()
#: A new model database has been created.
#: The model "Model-1" has been created.
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
#%%
# Inclusion creation and inserter script

from abaqus import *
from abaqusConstants import *
import numpy as np

def create_inclusion(radius = 0.5, depth = 5.0, model_name = 'Model-1'):
    size = radius**2
    # Create inclusion from sketch
    model  = mdb.models[model_name]
    name = "inclusion_left"
    
    model.Part(dimensionality=THREE_D, name=name, type=DEFORMABLE_BODY)
    part   = mdb.models[model_name].parts[name]
    sketch = model.Sketch(name=name, sheetSize=size)
    model.sketches[name].ArcByCenterEnds(center=(0.0, 0.0), point1=(0.0, radius), point2=(0.0, -radius), 
    direction=COUNTERCLOCKWISE)
    model.sketches[name].Line(point1=(0.0, radius), point2=(0.0, -radius))
    part.BaseSolidExtrude(sketch, 5.0)
    part = model.parts['inclusion_left']

    # Define geometric surfaces
    s = part.faces
    side1Faces = s.findAt(((0.0, 0.166667, 3.333333), ))
    part.Surface(side1Faces=side1Faces, name='int')
    s = part.faces
    side1Faces = s.findAt(((-0.064888, -0.495772, 3.333333), ))
    part.Surface(side1Faces=side1Faces, name='ext')

    # Create materials

    model.Material(name='hafnia')
    model.materials['hafnia'].Elastic(table=((280000.0, 0.28), ))
    model.HomogeneousSolidSection(name='oxide', material='hafnia', 
        thickness=None)
    cells = part.cells[:]
    #cell = cells[0]
    region = part.Set(cells=cells, name='semi_cyl')
    part.SectionAssignment(region=region, sectionName='oxide', offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)

    # Create assembly and mirror, this retains defined surfaces
    a = model.rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = model.parts['inclusion_left']
    a.Instance(name='inclusion_left-1', part=p, dependent=ON)
    a = model.rootAssembly
    a.LinearInstancePattern(instanceList=('inclusion_left-1', ), direction1=(1.0, 
        0.0, 0.0), direction2=(0.0, 1.0, 0.0), number1=2, number2=1, spacing1=0.5, 
        spacing2=1.0)
    a = model.rootAssembly
    a.rotate(instanceList=('inclusion_left-1-lin-2-1', ), axisPoint=(0.5, 0.0, 
        5.0), axisDirection=(0.0, 0.0, -5.0), angle=180.0)
    a = model.rootAssembly
    a.translate(instanceList=('inclusion_left-1-lin-2-1', ), vector=(-0.5, 0.0, 
        0.0))
    model.rootAssembly.features.changeKey(
    fromName='inclusion_left-1-lin-2-1', toName='inclusion_right-1')

    # Create fracture criterion

#%%
def pattern_inclusions(number, x, y, pattern, spacing=1, model_name = "Model-1"):
    if pattern == "series" and number!=1:
        num1, num2 = [number, 1]
        spa1, spa2 = [spacing, 1.0]
    elif pattern == "parallel" and number!=1:
        num1, num2 = [1, number]
        spa1, spa2 = [1.0, spacing]
    else:
        print "Something wrong with assignment"
    
    #First must create pattern in assembly
    model  = mdb.models[model_name]
    a = model.rootAssembly
    a.LinearInstancePattern(instanceList=('inclusion_right-1', 
    'inclusion_left-1'), direction1=(1.0, 0.0, 0.0), direction2=(0.0, 1.0, 
    0.0), number1=num1, number2=num2, spacing1=spa1, spacing2=spa2)

    # Translate
    instances = [i for i in a.instances.keys() if "inclusion" in i]

    a.translate(instanceList= tuple(instances), 
    vector=(x, y, 0.0))


    # TO DO : Naming generator 

def merge_inclusions(radius = 0.5, depth = 5.0, plate_name = 'plate', model_name = 'Model-1'):
    # Boolean cut on plate from list of parts
    model = mdb.models[model_name]
    a = model.rootAssembly
    cuts = []
    for inst in a.instances.keys():
        if inst != plate_name:
            print inst
            cuts.append(inst)#
    cut_name = "holey_plate"
    a.InstanceFromBooleanCut(name=cut_name, 
        instanceToBeCut=mdb.models[model_name].rootAssembly.instances[plate_name], 
        cuttingInstances=tuple(a.instances[i] for i in cuts), originalInstances=SUPPRESS)
    a.resumeFeatures(tuple(cuts))

    # Find the new surfaces created and name them
    names = ['hole_{}'.format(str(i)) for i in enumerate(cuts)]


def insert_czm(model_name="Model-1"):
    model  = mdb.models[model_name]
    # Create the interactions (AKA THE COHESIVE ZONE)
    mdb.models['Model-1'].ContactProperty('fracture')
    mdb.models['Model-1'].interactionProperties['fracture'].CohesiveBehavior(
        eligibility=ALL_NODES, defaultPenalties=OFF, table=((280000.0, 90000.0, 
        90000.0), ))
    mdb.models['Model-1'].interactionProperties['fracture'].Damage(initTable=((
        2300.0, 2300.0, 2300.0), ), useEvolution=ON, evolutionType=ENERGY, 
        evolTable=((40.0, ), ), useStabilization=ON, viscosityCoef=0.001)

    mdb.models['Model-1'].ContactProperty('decohesion')
    mdb.models['Model-1'].interactionProperties['decohesion'].Damage(initTable=((
        2050.0, 2050.0, 2050.0), ), useEvolution=ON, evolutionType=ENERGY, 
        evolTable=((40.0, ), ), useStabilization=ON, viscosityCoef=0.001)
    mdb.models['Model-1'].interactionProperties['decohesion'].CohesiveBehavior(
        eligibility=ALL_NODES, defaultPenalties=OFF, table=((210000.0, 90000.0, 
        90000.0), ))

    # first find fracture surfaces
    mdb.models["Model-1"].contactDetection(name = "CP", createStepName = "Step-1", includeCylindricalSphericalToric = OFF, includeSplineBased = OFF, interactionProperty= "fracture") # this function is amazing 

    # then find decohesion surfaces
    mdb.models["Model-1"].contactDetection(name = "CP", createStepName = "Step-1", includePlanar = OFF, interactionProperty= "decohesion")

    # Edit some contact properties
    for interaction in model.interactions.keys():
        mdb.models['Model-1'].interactions[interaction].setValues(
        initialClearance=0.0, datumAxis=None, adjustMethod=NONE, sliding=SMALL, 
        enforcement=SURFACE_TO_SURFACE, thickness=ON, 
        supplementaryContact=SELECTIVE, bondingSet=None)

    # 

def create_plate(plate_name = "plate", model_name = 'Model-1'):
    model  = mdb.models[model_name]
    s1 = model.ConstrainedSketch(name='__profile__', 
    sheetSize=200.0)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    s1.setPrimaryObject(option=STANDALONE)
    s1.rectangle(point1=(0.0, 0.0), point2=(100.0, 100.0))
    p = model.Part(name=plate_name, dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = model.parts[plate_name]
    p.BaseSolidExtrude(sketch=s1, depth=5.0)
    s1.unsetPrimaryObject()
    p = model.parts[plate_name]
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del model.sketches['__profile__']
    a = model.rootAssembly
    session.viewports['Viewport: 1'].setValues(displayedObject=a)
    session.viewports['Viewport: 1'].assemblyDisplay.setValues(interactions=OFF, 
        constraints=OFF, connectors=OFF, engineeringFeatures=OFF)
    a1 = model.rootAssembly
    p = model.parts[plate_name]
    a1.Instance(name=plate_name, part=p, dependent=ON)
#%%

def create_step(model_name = "Model-1"):
    model = mdb.models['Model-1']
    model.StaticStep(name='Step-1', previous='Initial', 
        timePeriod=220.0, maxNumInc=100000, initialInc=0.01, minInc=1e-09, 
        maxInc=0.5, nlgeom=ON)

# radius = 0.5
# depth = 5.0

# create_inclusion(radius, depth)

# x = 50.0
# y = 50.0
# number = 5
# pattern = 'parallel' # or series
# mean_free = 2 # this is from centre so do 
# spacing = 2 * radius + mean_free

# pattern_inclusions(number, x, y, pattern, spacing)

# create_plate(plate_name = "plate") # This function creates a plate which should be the microstructure

# create_step()

# merge_inclusions()

# insert_czm()

#%%
