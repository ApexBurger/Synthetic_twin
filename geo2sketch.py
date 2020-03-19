# Geo file to abaqus sketch 
from abaqus import *
from abaqusConstants import *




# Function to import .geo file - it simply converts points and lines into two dictionaries

def geo2sketch(scale = 1.0, model="Model-1"):
	import tkFileDialog
	import re

	file = tkFileDialog.askopenfilename()

	with open(file,'r') as geometry:
		lines = geometry.readlines()
		points = {}
		connects = {}
		n = 0
		n2 = 0
		for line in lines:
			if 'Point' in line:
				n += 1
				result = re.search('{(.*)}', line)
				new = result.group(1).split(',')
				points[n] = [float(i)*scale for i in new]
			if 'Line (' in line and 'Loop' not in line:
				n2 += 1
				result = re.search('{(.*)}', line)
				new = result.group(1).split(',')
				connects[n2] = [int(i) for i in new]

	# print(nodes, connects)
				

    # use abaqus syntax below to generate points then connect the lines

	s1 = mdb.models[model].ConstrainedSketch(name='__profile__', 
	sheetSize=200.0)
	g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
	s1.setPrimaryObject(option=STANDALONE)

	for point in points.values():
		s1.Spot(point=(point[0], point[1]))

	for line, nodes in connects.items():
		x1 = points[nodes[0]][0]
		y1 = points[nodes[0]][1]
		x2 = points[nodes[1]][0]
		y2 = points[nodes[1]][1]
		s1.Line(point1 = (x1, y1), point2 = (x2, y2))

	mdb.models[model].sketches.changeKey(fromName='__profile__', 
    toName='Microstructure_import')

	s1.unsetPrimaryObject()


# print(geo2sketch(file = 'n60-id2.geo'))