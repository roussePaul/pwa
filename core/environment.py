import networkx as nx
import numpy as np
if __name__ != '__main__':
	from gui import get_cmap
import random
import itertools
from scipy.spatial import Delaunay, Voronoi, voronoi_plot_2d
from matplotlib import pyplot as plt
import math

import utils
rect = lambda a,b: [(a[0],a[1]),(a[0],b[1]),(b[0],b[1]),(b[0],a[1])]

class Environment:
	#vectrices: list of points
	#elements: [(v1,v2,v3),(v2,v3,...),...] -> must be convex
	#regions: ["l1","l1","l3"]
	def __init__(self):
		self.vectrices = []
		self.elements = []
		self.regions = []
		self.colors = None

		self.center = {}
		self.reach_distance = {}
		self.show_reg = []

	def build(self,env):
		for l,e in env.items():
			elem = []
			for n in e:
				eq = [np.array_equal(x,n) for x in self.vectrices ]
				if True not in eq:
					idn = len(self.vectrices)
					self.vectrices.append(n)
				else:
					idn = eq.index(True) 
				elem.append(idn)

			self.elements.append(tuple(elem))
			self.regions.append(l)
		self.show_reg = self.regions

	def build_reachable_set(self):
		self.center = {}
		self.reach_distance = {}
		for elem in self.elements:
			b = self.get_baricenter(elem)
			self.center[elem] = b
			self.reach_distance[elem] = 0.5 *  self.get_distance_to_element(b,elem)

	def get_distance_to_element(self,p,elem):
		poly = [self.vectrices[e] for e in elem]
		return utils.distance_to_hull(p,poly)

	def build_color_map(self):
		s = set(self.regions)
		func = get_cmap(len(s))
		L = range(len(s))
		random.shuffle(L)
		self.colors = {r:func(i) for i,r in zip(L,s)}

	def is_element_reached(self,p,elem):
		b = self.center[elem]
		return np.linalg.norm(p-b)<self.reach_distance[elem]

	def get_point_in_region(self,region):
		elem = [e for e,r in zip(self.elements,self.regions) if r==region]
		if len(elem)>0:
			return self.get_baricenter(random.choice(elem))
		else:
			print "Region",region," not found"

	def get_elem_in_region(self,region):
		elem = [e for e,r in zip(self.elements,self.regions) if r==region]
		if len(elem)>0:
			return random.choice(elem)
		else:
			print "Region",region,"not found"

	def get_region(self,p):
		x = np.array(p)
		d = []
		for elem,r in zip(self.elements,self.regions):
			y = self.get_baricenter(elem)
			d.append((np.linalg.norm(x-y),r))
		d,r =  min(d, key=lambda x:x[0])
		return r

	def get_elem(self,p):
		x = np.array(p)
		d = []
		for elem,r in zip(self.elements,self.regions):
			y = self.get_baricenter(elem)
			d.append((np.linalg.norm(x-y),r,elem))
		d,r,elem =  min(d, key=lambda x:x[0])
		return elem

	def get_region_from_elem(self,elem):
		return self.regions[self.elements.index(elem)]
		
	def get_baricenter(self,elem):
		x = 0.0*self.vectrices[elem[0]]
		for n in elem:
			x += self.vectrices[n]
		return x / len(elem)


	def plot(self,plt):
		self.build_color_map()

		v = np.array(self.vectrices)
		elements = 	np.array([list(e) for e in self.elements])
		region_colors = list(itertools.chain.from_iterable([[r]*len(e) for r,e in zip(self.regions,self.elements)]))
		regions = {c:i for i,c in enumerate(set(region_colors))}
		ax = plt.gca()
		for e,r in zip(self.elements,self.regions):
			pts = [list(self.vectrices[n]) for n in e]
			if r=="c":
				ax.add_patch(plt.Polygon(pts, closed=True,fill=False,hatch='//',lw=0))
			elif r in self.show_reg:
				ax.add_patch(plt.Polygon(pts, closed=True,fill=True,color=self.colors[r]))
				if e in self.center:
					p = self.center[e]
					r = self.reach_distance[e]
					ax.add_patch(plt.Circle(p,radius=r,facecolor='none',edgecolor='k'))
			else:
				ax.add_patch(plt.Polygon(pts, closed=True,fill=False))

		for r in set(self.regions):
			try:
				if r in self.show_reg:
					p = self.get_point_in_region(r)
					ax.text(p[0], p[1], r, fontsize=15,horizontalalignment='center',verticalalignment='center',bbox={'facecolor':'white', 'alpha':0.5, 'pad':1, 'edgecolor':'none'})
			except TypeError,e:
				template = "An exception of type {0} occured. Arguments:\n{1!r}"
				message = template.format(type(ex).__name__, ex.args)
				print message,p,r

		return ax

	def show(self):
		self.plot(plt)
		plt.show()


	def plot_controls(self,plt,control_nodes,bs,scale=0.1,color='k',offset=np.array([0,0])):
		nodes = [n[0] for n in control_nodes.keys() if n[1]==bs]

		ax = plt.axes()
		for n,u in control_nodes.items():
			fs = n[0]
			start = self.get_baricenter(fs) + offset
			if u!=None:
				if np.all(u==0):
					ax.add_patch(plt.Circle((start[0],start[1]), radius=scale, color=color, fill=True))
				else:
					x = scale * u/np.linalg.norm(u)
					ax.arrow(start[0], start[1], x[0],x[1], head_width=0.01, head_length=0.02, fc=color, ec=color)
			else:
				ax.add_patch(plt.Rectangle((start[0],start[1]),0.03,0.03,facecolor=color,edgecolor=color))

	def show_controls(self,plt,nodes,filename):
		buchi_states = sorted(set([n[1] for n in nodes.keys()]))
		plt.clf()
		self.plot(plt)
		color = ['b','r','k','c']
		for i,bs in enumerate(buchi_states):
			c = color[i%len(color)]
			print bs,"\t",c
			self.plot_controls(plt,nodes,bs,scale=0.1,color=c)
		plt.savefig(filename)


class DelaunayEnvironment(Environment):
	def __init__(self,points,area,labels):
		Environment.__init__(self)
		
		self.area = area
		self.area_hull = Delaunay(area)
		self.points = points
		self.regions = labels

		self.get_voronoi()
		self.build_color_map()
		self.build_reachable_set()	

	def add_symmetric(self):
		sym = []
		a = list(self.area)
		for u,v in zip(a,a[1:]+a[0:1]):
			d = (u-v)
			n = np.array([d[1],-d[0]])
			n = n/np.linalg.norm(n)
			sym += [p-2*n*(np.dot(n,p-u)) for p in self.points]
		self.points = np.array(list(self.points) + sym)
	
	def get_voronoi(self):
		self.add_symmetric()
		vor = Voronoi(self.points)
		index_inside_vectrices = set([])
		for i,v in enumerate(vor.vertices):
			if self.in_area(v):
				index_inside_vectrices.add(i)
		elements = []
		for r in vor.regions:
			if r and set(r).issubset(index_inside_vectrices):
				elements.append(r)
		
		index_inside_vectrices = list(index_inside_vectrices)
		reindex = {n:i for i,n in enumerate(index_inside_vectrices)}

		vectrices = [vor.vertices[n] for n in index_inside_vectrices]
		
		elements = [tuple([reindex[n] for n in e]) for e in elements]

		self.elements = elements
		self.vectrices = vectrices

	def in_area(self,p):
		return self.area_hull.find_simplex(p)>=0

class MonotoneEnvironment(Environment):
	def __init__(self,monotone_system):
		Environment.__init__(self)
		
		self.elements = [tuple([e[0],e[1],e[3],e[2]])+e[4:] for e in monotone_system.elements]
		self.vectrices = monotone_system.vectrices
		self.regions = [str(i) for i in range(len(self.elements))]

		self.build_color_map()

class GridEnvironment:
	def __init__(self,n,m,area):
		self.n = n
		self.m = m
		self.area = area

	def plot(self,plt):
		ax = plt.gca()
		for x in np.linspace(self.area[0],self.area[1],self.n):
			ax.plot([x,x],[self.area[2],self.area[3]],'k')
		for y in np.linspace(self.area[2],self.area[3],self.m):
			ax.plot([self.area[0],self.area[1]],[y,y],'k')
	def get_elem(self,p):
		print "Not implemented",p

if __name__ == '__main__':
	area = np.array(rect([-1,-1],[1,1]))
	r = 0.3
	a = 0.5*r
	b = math.sqrt(3)/2*r
	X1 = np.arange(-1,1,2*b)
	X2 = np.arange(b-1,1,2*b)
	Y1 = np.arange(-1,1,2*a)
	Y2 = np.arange(a-1,1,2*a)
	pts1 = np.dstack(np.meshgrid(X1, Y1)).reshape(-1, 2)
	pts2 = np.dstack(np.meshgrid(X2, Y2)).reshape(-1, 2)
	pts = np.concatenate((pts1,pts2),axis=0)

	labels = ["a"]*pts.shape[0]
	env = DelaunayEnvironment(pts,area,labels)
