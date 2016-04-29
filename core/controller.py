import simulator
import numpy as np
import random
import time
import itertools
from automata import FTS
def sat(x,m):
	return x if abs(x)<m else x/abs(x)*m

class ControlledSimObj(simulator.ContinuousSimObj):
	def __init__(self,env):
		simulator.ContinuousSimObj.__init__(self)
		self.speed = 3.0
		self.env = env
		self.ref_point = np.array([1,1])
		self.ref_elem = random.choice(self.env.elements)
		self.goto(self.ref_elem)

	def goto(self,to_elem):
		self.ref_elem = to_elem
		self.ref_point = self.env.get_baricenter(to_elem)

	def is_done(self):
		return self.env.is_element_reached(self.state,self.ref_elem)

	def cont_sim(self,dt):
		d = np.linalg.norm(self.ref_point-self.state)
		if d<1e-6:
			return self.state*0
		n = (self.ref_point-self.state)/d
		self.u = sat(d,1)*self.speed*n

		simulator.ContinuousSimObj.cont_sim(self,dt)

	def get_FTS(self):
		nodes = [(e,{'label':set([r]),'weight':1.0}) for e,r in zip(self.env.elements,self.env.regions)]
		L = self.env.elements
		E = list(itertools.permutations(L,2)) + [(l,l) for l in L]
		edges = [(u,v,{'weight':1.0}) for u,v in E]

		symbols = set(self.env.regions)
		print self.ref_elem
		fts = FTS(symbols,set([self.ref_elem]))

		fts.add_nodes_from(nodes)
		fts.add_edges_from(edges)
		return fts
