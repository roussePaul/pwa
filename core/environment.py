import networkx as nx
import numpy as np
from gui import get_cmap
import random
import itertools

rect = lambda a,b: [(a[0],a[1]),(a[0],b[1]),(b[0],b[1]),(b[0],a[1])]

class Environment:
	#vectrices: list of points
	#elements: [(v1,v2,v3),(v2,v3,...),...] -> must be convex
	#regions: ["l1","l1","l3"]

	def __init__(self,env):
		self.vectrices = []
		self.elements = []
		self.regions = []
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

		self.colors = get_cmap(len(set(self.regions)))

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
				print r
				ax.add_patch(plt.Polygon(pts, closed=True,fill=False,hatch='//'))
			else:
				ax.add_patch(plt.Polygon(pts, closed=True,fill=True,color=self.colors(regions[r])))

		for r in set(self.regions):
			p = self.get_point_in_region(r)
			ax.text(p[0], p[1], r, fontsize=15,horizontalalignment='center',verticalalignment='center',bbox={'facecolor':'white', 'alpha':0.5, 'pad':1, 'edgecolor':'none'})


		return ax
