import numpy as np
import copy
import networkx as nx
from Queue import *
import mesh
from matplotlib.pyplot import *
import math
import itertools


E = lambda x,y: frozenset([x,y])
def distance(p1,p2):
	return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

class Controller:
	def __init__(self,environment_mesh):
		self.mesh = environment_mesh
		self.vectrices = np.vstack(self.mesh.points) 
		self.elements = np.vstack(self.mesh.elements).tolist()
		self.elements = [tuple(e) for e in self.elements]
		self.regions = np.hstack(self.mesh.element_attributes)
		self.edges = self.compute_edges()
		self.normals = self.compute_normals()
		self.transition_graph = self.get_transition_graph()
		self.path_graph = None
		self.hysteresis_transitions = None
		self.orientations = self.compute_orientations()
		self.orientations_path = copy.deepcopy(self.orientations)

		self.input = [0.0,0.0]
		self.output = [0.0,0.0]
		self.prev_elem = None
		self.hysteresis = 0.01
		self.affine_feedback = dict()

		self.eq_map = None
		self.control_values = None

	def build(self,to_region):
		obstacles = set(range(4))-set([to_region,3])
		self.path_graph,self.hysteresis_transitions  = self.find_paths_2_region(to_region,list(obstacles))
		self.compute_controller_continuous()

	def set_input(self,x):
		self.input = copy.copy(x)

	def init_controller(self):
		self.prev_elem = self.find_closest_triangle(self.input)

	def update(self):
		self.prev_elem = self.find_triangle(self.input)
		self.compute_feedback(self.prev_elem,self.input)

	def compute_feedback(self,elem,p):
		if elem==None:
			return None
		F = self.affine_feedback[elem][0]
		g = self.affine_feedback[elem][1]
		x = np.array([p])
		self.output = np.dot(F,np.transpose(x)) + g

	def get_output(self):
		return self.output

	def get_transition_graph(self):
		G=nx.Graph()
		for (e,r) in zip(self.elements,self.regions):
			G.add_node(tuple(e), pos=self.get_baricenter(e),region=r)
		for n in G.nodes():
			for m in G.nodes():
				if m!=n and len(set(n).intersection(m))>=2:
					G.add_edge(m,n)
		return G

	def get_baricenter(self,e):
		px = sum([self.vectrices[ide][0] for ide in e])/3
		py = sum([self.vectrices[ide][1] for ide in e])/3
		return [px,py]

	def get_current_state(self):
		pass

	def compute_edges(self):
		edges = set()
		for e in self.elements:
			l = list(e)
			X = zip(l,[l[-1]]+l[:-1])
			X = [E(u,v) for (u,v) in X]
			edges.update(X)
		return edges

	def compute_normals(self):
		n = dict()
		for (p,q) in self.edges:
			v = self.vectrices[p]-self.vectrices[q]
			normal = [v[1],-v[0]]
			_n = normal/np.linalg.norm(normal)
			n[E(p,q)] = _n
		return n

	def compute_orientations(self):
		orientations = dict()
		for e in self.elements:
			edge_orient = dict()
			N = len(e)
			for i in range(0,N):
				en = e[i]
				ec = e[(i+1)%N]
				ep = e[(i+2)%N]
				x = self.vectrices[ep]-self.vectrices[ec]
				p = np.dot(self.normals[E(ec,en)],x)
				if p<0:
					edge_orient[E(ec,en)] = -self.normals[E(ec,en)]
				else:
					edge_orient[E(ec,en)] = self.normals[E(ec,en)]
			orientations[e] = edge_orient
		return orientations

	def compute_orientation_path(self):
		for (u,v) in self.path_graph.edges():
			if u!=v:
				edge = tuple(set(list(u)).intersection(set(list(v))))
				edge = E(*edge)
				self.orientations_path[u][edge] = -(self.orientations_path[u][edge].copy())

	def compute_controller(self):
		self.compute_orientation_path()
		for e in self.elements:
			self.affine_feedback[e] = self.compute_afb(e)

	def compute_afb(self,elem):
		u = []
		pts = [self.vectrices[i] for i in elem]
		_pts = np.array(pts)
		N = _pts.shape[1]+1
		
		normals = self.orientations_path[elem]
		for i in range(0,N):
			en = elem[(i+1)%N]
			ep = elem[(i-1)%N]
			e = elem[i]

			v = normals[E(en,e)]
			w = normals[E(ep,e)]
			_u = (v+w)/2
			_u = _u/np.linalg.norm(_u)
			u.append(_u)
		U = np.transpose(np.array(u))
		V = np.concatenate((np.transpose(_pts), np.ones((1,N))), axis=0)
		Fg = np.dot(U,np.linalg.inv(V))
		F = Fg[:2,:2]
		g = np.transpose(np.array([Fg[:,2]]))
		return (F,g)


	def on_the_same_path(self,e,f):
		return nx.has_path(self.path_graph,e,f) or nx.has_path(self.path_graph,f,e)

	def get_equality_map(self):
		node_to_elements = {k:[] for k in range(len(self.vectrices))}
		for e in self.elements:
			for t in e:
				node_to_elements[t].append(e)
		indep_control_values = {}
		N = 0
		for e in self.elements:
			for n in e:
				found = False
				for f in node_to_elements[n]:
					if f!=e and self.on_the_same_path(e,f):
						if (f,n) in indep_control_values.keys():
							if found == False:
								indep_control_values[(e,n)] = indep_control_values[(f,n)]
								found = True
							else:
								new_idn = indep_control_values[(e,n)]
								old_idn = indep_control_values[(f,n)]
								for node,_id in indep_control_values.items():
									if _id == old_idn:
										indep_control_values[node] = new_idn
				if found==False:
					indep_control_values[(e,n)] = N
					N += 1

		idn = list(set(indep_control_values.values()))
		new_idn = range(len(idn))
		remplace_id = {n:m for n,m in zip(idn,new_idn)}
		indep_control_values_new = {n:remplace_id[m] for n,m in indep_control_values.items()}
		return indep_control_values_new


	def compute_controller_continuous(self):
		print "Path..."
		self.compute_orientation_path()
		print "Constraints..."
		self.eq_map = self.get_equality_map()

		idx = set(self.eq_map.values())
		N = len(idx)
		sup = np.zeros((N,1))-np.inf
		inf = np.zeros((N,1))+np.inf
		C = np.zeros((N,1))
		alpha = np.zeros((N,1))
		constraints = {i:[] for i in idx}
		for (elem,ec),idn in self.eq_map.iteritems():
			normals = self.orientations_path[elem]
			(en,ep) = tuple(set(list(elem)) - set([ec]))
			v = normals[E(en,ec)]
			w = normals[E(ep,ec)]
			a = np.arctan2(v[1],v[0])
			b = np.arctan2(w[1],w[0])

			sup[idn] = max(a+np.pi/2,b+np.pi/2,sup[idn])%(2*np.pi)
			inf[idn] = min(a-np.pi/2,b-np.pi/2,inf[idn])%(2*np.pi)
			constraints[idn].append(a)
			constraints[idn].append(b)

		print "Resolve constraints..."
		delta  = 1.0e-4
		alpha = np.zeros((N,1))
		beta = np.zeros((N,1)) + np.pi/2 + delta
		for i,a in constraints.items():
			alpha[i] = a[0]
			for j in range(1,len(a)):
				_s1 = alpha[i]+beta[i]
				_s2 = a[j]+np.pi/2 + delta
				_i1 = alpha[i]-beta[i]
				_i2 = a[j]-np.pi/2 - delta
				if _s1<_i2:
					_s2 -= 2*np.pi
					_i2 -= 2*np.pi
					if _i1>_s2:
						beta[i] = -1
						break
				if _s2<_i1:
					_s2 += 2*np.pi
					_i2 += 2*np.pi
					if _s1<_i2:
						beta[i] = -1
						break
				if _i2<_i1 and _s1<_s2:
					continue
				if _i2>_i1 and _s1<_s2:
					alpha[i] = (_s1+_i2)/2.0
					beta[i] = (_s1-_i2)/2.0
				if _i2<_i1 and _s1>_s2:
					alpha[i] = (_s2+_i1)/2.0
					beta[i] = (_s2-_i1)/2.0

		A = alpha
		C = beta>0


		X = C*np.cos(A)
		Y = C*np.sin(A)

		print "Affine controller computation..."
		control_values = {}
		for (elem,ec),idn in self.eq_map.iteritems():
			control_values[(elem,ec)]  = np.array([X[idn],Y[idn]])


		for e in self.elements:
			u1 = control_values[(e,e[0])]
			u2 = control_values[(e,e[1])]
			u3 = control_values[(e,e[2])]
			U = np.concatenate((u1,u2,u3),axis=1)
			pts = [self.vectrices[i] for i in e]
			_pts = np.array(pts)
			V = np.concatenate((np.transpose(_pts), np.ones((1,3))), axis=0)
			Fg = np.dot(U,np.linalg.inv(V))
			F = Fg[:2,:2]
			g = np.transpose(np.array([Fg[:,2]]))
			self.affine_feedback[e] = (F,g)

		self.control_values = control_values


	def get_nodes_from_region(self,region):
		return [n for (n,attr) in self.transition_graph.nodes(data=True) if attr['region']==region]

	def find_paths_2_region(self,region_target,obstacles):
		G = nx.Graph()
		G.add_nodes_from(self.transition_graph.nodes(data=True))

		for (u,v) in self.transition_graph.edges():
			w = distance(u,v)
			G.add_edge(u,v,weight=w)

#		for r in obstacles:
#			obstacle_nodes = self.get_nodes_from_region(r)
#			G.remove_nodes_from(obstacle_nodes)

		H = nx.DiGraph()
		H.add_nodes_from(G.nodes(data=True))

		visited_nodes = []

		region_node_target = self.get_nodes_from_region(region_target)

		node_to_visit = [(target,0.0) for target in region_node_target]
		dist = nx.get_node_attributes(G,'weight')
		positions = nx.get_node_attributes(G,'pos')
		regions = nx.get_node_attributes(G,'regions')
		while len(node_to_visit)>0:
			node_to_visit = sorted(node_to_visit,key = lambda x: x[1])
			current_node,current_distance = node_to_visit.pop(0)
			neighbors = G.neighbors(current_node)
			for n in neighbors:
				if n not in visited_nodes:
					H.add_edge(n,current_node)
					node_to_visit.append((n,current_distance+
						distance(positions[n],positions[current_node])))
					visited_nodes.append(n)

		sG = [tuple(sorted(e)) for e in G.edges()]
		sH = [tuple(sorted(e)) for e in H.edges()]
		deleted_transitions = [e for e in sG if e not in sH]
		return H,deleted_transitions

	def get_element_vectrices(self,elem):
		return np.array([self.vectrices[e] for e in elem])

	def distance_to_triangle(self,elem,p):
		d = []
		normals = self.orientations[elem]
		x = np.array(p)
		N = 3
		r = []
		for i in range(0,N):
			en = elem[(i+1)%N]
			e = elem[i]
			n = normals[E(en,e)]
			v = self.vectrices[e]
			d = np.dot(x-v,n)
			r.append(d)
		proj = [c for c in r if c>0]
		h = len(proj)
		if h==3:
			return 1
		elif h==2:
			return [c for c in r if c<=0][0]
		elif h==1:
			return -min([distance(self.vectrices[e],x) for e in elem])
 
	def find_closest_triangle(self,p):
		for e in self.elements:
			if self.distance_to_triangle(e,p)>=0:
				return e

	def find_closest_edge_in_triangle(self,elem,p):
		d = [0]*3
		x = np.array(p)
		for i in range(3):
			d[i] = np.linalg.norm(x-self.vectrices[elem[i]])
		idn = np.argmin(d)
		return elem[idn]

	def find_triangle(self,p):
		current_triangle = self.find_closest_triangle(p)

		if tuple(sorted([self.prev_elem,current_triangle])) in self.hysteresis_transitions:
			if abs(self.distance_to_triangle(self.prev_elem,p))<self.hysteresis:
				return self.prev_elem
		return current_triangle

def draw_quiver(controller):
	N = 10
	f = lambda u,v,a: a*(u-v) + v
	X = []
	Y = []
	for e in controller.elements:
		for edge,normal in controller.orientations[e].iteritems():
			delta = normal*1.0e-5
			(u,v) = tuple(edge)
			for t in np.linspace(0.0,1.0,N):
				b = f(controller.vectrices[v],controller.vectrices[u],t) + delta
				X.append(b[0])
				Y.append(b[1])

	X = np.array(X)
	Y = np.array(Y)
	U = X.copy()
	V = Y.copy()
	for i in range(len(X)):
		x = [X[i],Y[i]]
		controller.set_input(x)
		controller.init_controller()
		controller.update()
		u = controller.get_output()
		U[i] = u[0]
		V[i] = u[1] 
		
	quiver(X,Y,U,V)

def draw_orientation(controller,elem):
	X = []
	Y = []
	U = []
	V = []
	for (e,n) in controller.orientations[elem].iteritems():
		e = list(e)
		p = (controller.vectrices[e[0]]+controller.vectrices[e[1]])/2.0
		X.append(p[0])
		Y.append(p[1])
		U.append(n[0])
		V.append(n[1])

	quiver(X,Y,U,V)


def mouse_move(controller,ax,event):
	if not event.inaxes:
		return

	x, y = event.xdata, event.ydata

	x = [x,y]
	controller.set_input(x)
	controller.update()
	u = controller.get_output()

	if controller.prev_elem==None:
		return 
	p =  [controller.vectrices[e] for e in controller.prev_elem]
	p = p + [p[0]] 
	p = np.array(p)
	ax[0].set_data(p[:,0],p[:,1])

	closest_node = controller.find_closest_edge_in_triangle(controller.prev_elem,x)
	eq_nodes = [e for e,n in controller.eq_map.keys() if n==closest_node and controller.eq_map[(e,n)] == controller.eq_map[(controller.prev_elem,closest_node)]]

	p = [controller.get_baricenter(e) for e in eq_nodes]
	l = []
	z = controller.vectrices[closest_node]
	for y in p:
		l.append(z)
		l.append(y)
	p = np.array(l)
	ax[1].set_data(p[:,0],p[:,1])

	draw()

class MouseEvent:
	def __init__(self,controller):
		self.controller = controller


if __name__ == '__main__':
	
	print "Mesh..."
	mesh = mesh.get_mesh_example()
	print "Controller..."
	controller = Controller(mesh)
	print "Build..."
	controller.build(2)

	colors = controller.regions

	elements = np.array([list(e) for e in controller.elements])
	tripcolor(controller.vectrices[:,0], controller.vectrices[:,1], elements[:,:3],facecolors=colors,edgecolors ='k')

	plot(controller.vectrices[:,0], controller.vectrices[:,1], 'ko') # Manually plot all points including the ones at the midpoints of triangle faces


	px = [p['pos'][0] for (n,p) in controller.path_graph.nodes(data="pos")]
	py = [p['pos'][1] for (n,p) in controller.path_graph.nodes(data="pos")]
	pos = {n:p['pos'] for (n,p) in controller.path_graph.nodes(data="pos")}

	plot(px,py, 'co')
	nx.draw_networkx_edges(controller.path_graph,pos=pos)


	H = nx.Graph()
	H.add_nodes_from(controller.path_graph.nodes(data=True))
	H.add_edges_from(controller.hysteresis_transitions)
	nx.draw_networkx_edges(H,pos=pos,edge_color='r',alpha=0.5)

	curr_triangle, = plot([0],[0],'b-',linewidth=2.0)
	curr_node, = plot([0],[0],'g-',linewidth=1.5)
	print "Draw quiver..."
	draw_quiver(controller)

	connect('motion_notify_event', lambda event,ax=[curr_triangle,curr_node],controller=controller: mouse_move(controller,ax,event))
	show()
