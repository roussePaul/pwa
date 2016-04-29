import sys
sys.path.append("../core")

from automata import *
from agents import *
from gui import *
from environment import *
from simulator import *
from monotone_system import *

import time
import itertools
import networkx as nx
import random

def flatten(l):
    for el in l:
        if isinstance(el, list) and not isinstance(el, basestring):
            for sub in flatten(el):
                yield sub
        else:
            yield el

def init():
	mesh = MonotoneSystem()
	mesh.rectangular_mesh((0,0),(1,1),(10,10))
	a = 0.1
	inputs = [np.array([0,0]),np.array([0,a]),np.array([a,0]),np.array([-a,0]),np.array([0,-a])]
	fts = mesh.compute_FTS(inputs)



	init_elem = list(mesh.collision(np.array([0,0]),np.array([0.2,0.2])))
	avoid_elem = list(mesh.collision(np.array([0.4,0.4]),np.array([0.6,0.6])))
	final_elem = list(mesh.collision(np.array([0.8,0.8]),np.array([1.0,1.0])))

	init = random.choice(init_elem)
	fts.graph['initial'] = set([init])
	final = random.choice(final_elem)

	predecessor = nx.predecessor(fts,final)
	paths = [[k]+v for k,v in predecessor.items()]
	used_edges = list(flatten([ zip(p[1:],p[0:-1]) for p in paths if len(p)>1 ]))
	used_edges = list(flatten([ zip(p[0:-1],p[1:]) for p in paths if len(p)>1 ]))
	used_edges.append((final,final))
	fts.remove_edges_from(set(fts.edges()).difference(set(used_edges)))
	fts[final][final]['control'] = np.array([0,0])
	fts[final][final]['label'] = str(np.array([0,0]))

	#fts.show("lkjlkj")

	env = MonotoneEnvironment(mesh)

	for i,elem in enumerate(mesh.elements):
		if elem in init_elem:
			env.regions[i] = "i"
		if elem in avoid_elem:
			env.regions[i] = "c"
		if elem in final_elem:
			env.regions[i] = "f"

	print env.regions
	q1 = Quad("q1","i",env)
	planner = MonotonePlanner(q1,mesh,fts)


	sim = Simulator()

	sim.add("q1",q1)
	sim.add("q1_planner",planner)
	
	return sim,env

if __name__ == '__main__':
	sim,env = init() 
	gui = GUI(env,sim)
	sim.start_simulation(realtime=True)
	#sim.wait_for_keyboard_interrupt()
	gui.start()
