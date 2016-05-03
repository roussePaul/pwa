import shapely
import numpy as np
from scipy.optimize import linprog
import itertools
import networkx as nx
import automata

def less(x,y):
	return all(x<y)
def less_eq(x,y):
	return all(x<=y)

class MonotoneSystem():
	def __init__(self,out_state=False):
		self.vectrices = []
		self.elements = []
		self.faces = []

		self.area = None
		self.out_state = out_state
		self.out = "out"

	def rectangular_mesh(self,xmin,xmax,points):
		self.area = [xmin,xmax]

		n = len(xmin)
		l = []
		dx = []
		for m,M,p in zip(xmin,xmax,points):
			l.append(np.linspace(m,M,p))
			dx.append( (M-m)/(p-1.0) )
		vectrices = list(itertools.product(*l))
		vectrices = [np.array(list(x)) for x in vectrices]
		vectrices = np.array(vectrices)

		element = set([])
		epsilon = 1e-4
		t = list(itertools.product(*[[0,d] for d in dx]))
		t = [np.array(list(x)) for x in t]
		print t
		elem_normals = []
		for i in range(n):
			norm = [0]*n
			norm[i] = 1
			elem_normals.append(np.array(norm))
			norm = [0]*n
			norm[i] = -1
			elem_normals.append(np.array(norm))

		elem_faces = {k:[] for k in range(len(elem_normals))}
		baricenter = sum(t)*1.0/len(t)
		for idn,norm in enumerate(elem_normals):
			for idp,p in enumerate(t):
				if np.dot(norm,baricenter-p)>0:
					elem_faces[idn].append(idp)

		for v in vectrices:
			elem = []
			try:
				for translate in t:
					pts = v + translate
					iv = np.argwhere(np.all(abs(vectrices-pts)<epsilon, axis=1))
					elem.append(int(iv))
				element.add(tuple(elem))
			except TypeError,ex:
				template = "An exception of type {0} occured. Arguments:\n{1!r}"
				message = template.format(type(ex).__name__, ex.args)
				print message

		self.elements = list(element)
		self.vectrices = list(vectrices)


	def get_cell(self,p):
		for elem in self.elements:
			if self.is_in_cell(elem,p):
				return elem
		return None

	def is_in_cell(self,elem,p):
		return less(self.vectrices[elem[0]],p) and less(p,self.vectrices[elem[-1]])

	def is_out(self,p):
		return not (less_eq(self.area[0],p) and less_eq(p,self.area[1]))

	def collision(self,xmin,xmax):
		colliding_elements = set([])
		in_rect = lambda x,m,M: less(m,x) and less(x,M)
		for elem in self.elements:
			ymin = self.vectrices[elem[0]]
			ymax = self.vectrices[elem[-1]]

			if 	in_rect(ymin,xmin,xmax) or in_rect(ymax,xmin,xmax) or in_rect(xmin,ymin,ymax) or in_rect(xmax,ymin,ymax) or (less(xmin,ymax) and less(ymin,xmax)):
				colliding_elements.add(elem)
			if self.out_state and (self.is_out(xmin) or self.is_out(xmax)):
				colliding_elements.add(self.out)
				#print "Out",self.area[0],"<",xmin,",",xmax,"<=",self.area[1]
		return colliding_elements

	def find_next_cells(self,x,u):
		new_cell = []
		for e in [x[0],x[-1]]:
			v = self.vectrices[e]

			w = v + u
			new_cell.append(w)
		return self.collision(new_cell[0],new_cell[-1])

	def compute_FTS(self,input_space_partition):
		G = automata.FTS()
		for j,u in enumerate(input_space_partition):
			print j,u
			edges = set([])
			for i,x in enumerate(self.elements):
				next_elem = self.find_next_cells(x,u)
				for n in next_elem:
					edges.add((x,n))

			if self.out_state:
				edges.add((self.out,self.out))
			for e in edges:
				if e in G.edges():
					if str(u) not in G[e[0]][e[1]]['label']:
						G[e[0]][e[1]]['label'].add(str(u))
						G[e[0]][e[1]]['control'][str(u)] = u
				else:
					G.add_edge(e[0],e[1],label=set([str(u)]),control={str(u):u})
		return G

	def get_linear_system(self,elem):
		return np.eye(2),np.eye(2)


	def is_loop_fair(self,controls):
		for u in controls:
			if np.all(u==0):
				return False
		A = np.transpose(np.array(controls))
		(M,N) = A.shape
		c = -np.ones(N)
		b = np.zeros((M,1))
		eps = 1e-4
		res = linprog(c, A_eq=A, b_eq=b, bounds=tuple([(eps,1.0)]*N))
		return not res['success']

	def is_graph_fair(self,graph):
		control_dict = nx.get_node_attributes(graph,'control')
		for cycle in nx.simple_cycles(graph):
			controls = [control_dict[n] for n in cycle]
			if self.is_loop_fair(controls) == False:
				return False
		return True

def test_linprog():
	controls= [np.array([1,0]),np.array([-1,0]),np.array([0,1])]
	A = np.transpose(np.array(controls))
	(M,N) = A.shape
	c = -np.ones(N)
	b = np.zeros((M,1))
	eps = 1e-4
	res = linprog(c, A_eq=A, b_eq=b, bounds=tuple([(eps,1.0)]*N))
	print res
	print res['success']

def test_monotone_system():
	import matplotlib.pyplot as plt
	mesh = MonotoneSystem()
	mesh.rectangular_mesh((-3,-3),(3,3),(10,10))
	xmin = np.array([-2.0,-1.])
	xmax = np.array([-1.,-0.])
	collide = mesh.collision(xmin,xmax)
	inputs = [np.array([0,1]),np.array([1,0]),np.array([-1,0]),np.array([0,-1])]
	mesh.compute_FTS(inputs)

	rect = lambda a,b: np.array([[a[0],a[1]],[a[0],b[1]],[b[0],b[1]],[b[0],a[1]],[a[0],a[1]]])
	pts = rect(xmin,xmax)
	plt.plot(pts[:,0],pts[:,1],'g')

	v = np.array(mesh.vectrices)
	elements = 	np.array([list(e) for e in mesh.elements])
	ax = plt.gca()
	for e in mesh.elements:
		pts = [list(mesh.vectrices[n]) for n in e]
		if e in collide:
			c = 'b'
		else:
			c = 'r'
		ax.add_patch(plt.Polygon(pts, closed=True,fill=True,color=c))

	ax.set_xlim([-3,3])
	ax.set_ylim([-3,3])

	plt.show()

if __name__ == '__main__':
	test_linprog()