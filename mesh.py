from numpy import *
from matplotlib.pyplot import *
from meshpy.triangle import MeshInfo, build

# Utility function to create lists of the form [(1,2), (2,3), (3,4),
#(4,1)], given two numbers 1 and 4
from itertools import islice, cycle
from six.moves import range
from six.moves import zip
loop = lambda a, b: list(zip(list(range(a, b)), islice(cycle(list(range(a, b))), 1, None)))
rect = lambda a,b: [(a[0],a[1]),(a[0],b[1]),(b[0],b[1]),(b[0],a[1])]


def get_mesh_example():
	area = rect((0.,0.),(1.,1.))
	init = rect((0.1,0.1),(0.2,0.2))
	goal1 = rect((0.8,0.8),(0.9,0.9))
	goal2 = rect((0.2,0.8),(0.3,0.9))

	env = area + goal1 + goal2 + init

	info = MeshInfo()
	info.set_points(env)
	info.set_facets(loop(0,4) + loop(4,8) + loop(8,12) + loop(12,16), facet_markers = [0]*4 + [1]*4 + [2]*4 + [3]*4) # Create 8 facets and apply markers 1-8 on them
	info.regions.resize(4)
	s = 0.05
	info.regions[0] = [0.15, 0.15, 0, s] # Replace 0.1 by a smaller value to produce a finer mesh
	info.regions[1] = [0.25, 0.85, 1, s] # Fourth item specifies maximum area of triangles as a region attribute
	info.regions[2] = [0.85, 0.85, 2, s] # Replace 0.1 by a smaller value to produce a finer mesh
	info.regions[3] = [0.5, 0.5, 3, s] # Fourth item specifies maximum area of triangles as a region attribute

	mesh = build(info, volume_constraints=True, attributes=True,
	generate_faces=True, min_angle=20, mesh_order=1)
	return mesh

if __name__ == "__main__":
	mesh = get_mesh_example()
	pts = vstack(mesh.points) # (npoints, 2)-array of points
	elements = vstack(mesh.elements) # (ntriangles, 6)-array specifying element connectivity

	colors = vstack(mesh.element_attributes)

	# Matplotlib's Triangulation module uses only linear elements, so use only first 3 columns when plotting
	colors = colors[:,0]

	tripcolor(pts[:,0], pts[:,1], elements[:,:3],facecolors=colors,edgecolors ='k')

	plot(pts[:,0], pts[:,1], 'ko') # Manually plot all points including the ones at the midpoints of triangle faces


	print colors
	show()
