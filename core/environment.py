import networkx as nx
import numpy as np
from gui import get_cmap
import random
import itertools
from scipy.spatial import Delaunay, Voronoi, voronoi_plot_2d

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
		self.colors = get_cmap(len(set(self.regions)))

	def is_element_reached(self,p,elem):
		b = self.center[elem]
		return np.linalg.norm(p-b)<self.reach_distance[elem]

	def get_point_in_region(self,region):
		elem = [e for e,r in zip(self.elements,self.regions) if r==region]
		if len(elem)>0:
			return self.get_baricenter(random.choice(elem))
		else:
			print "Region not found"

	def get_elem_in_region(self,region):
		elem = [e for e,r in zip(self.elements,self.regions) if r==region]
		if len(elem)>0:
			return random.choice(elem)
		else:
			print "Region not found"

	def get_region(self,p):
		x = np.array(p)
		d = []
		for elem,r in zip(self.elements,self.regions):
			y = self.get_baricenter(elem)
			d.append((np.linalg.norm(x-y),r))
		d,r =  min(d, key=lambda x:x[0])
		return r

	def get_region_from_elem(self,elem):
		return self.regions[self.elements.index(elem)]
		
	def get_baricenter(self,elem):
		x = np.array([0.0,0.0])
		for n in elem:
			x += self.vectrices[n]
		return x / len(elem)


	def plot(self,plt):
		v = np.array(self.vectrices)
		elements = 	np.array([list(e) for e in self.elements])
		region_colors = list(itertools.chain.from_iterable([[r]*len(e) for r,e in zip(self.regions,self.elements)]))
		regions = {c:i for i,c in enumerate(set(region_colors))}
		ax = plt.gca()
		for e,r in zip(self.elements,self.regions):
			pts = [list(self.vectrices[n]) for n in e]
			if r=="obstacle":
				ax.add_patch(plt.Polygon(pts, closed=True,fill=False,hatch='//'))
			else:
				ax.add_patch(plt.Polygon(pts, closed=True,fill=True,color=self.colors(regions[r])))
				p = self.center[e]
				r = self.reach_distance[e]
				ax.add_patch(plt.Circle(p,radius=r,facecolor='none',edgecolor='k'))

		for r in set(self.regions):
			p = self.get_point_in_region(r)
			ax.text(p[0], p[1], r, fontsize=15,horizontalalignment='center',verticalalignment='center',bbox={'facecolor':'white', 'alpha':0.5, 'pad':1, 'edgecolor':'none'})


		return ax

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

if __name__ == '__main__':
	area = np.array(rect([-1,-1],[1,1]))
	DelaunayEnvironment()