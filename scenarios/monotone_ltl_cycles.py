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
	mesh = MonotoneSystem(out_state=True)
	mesh.rectangular_mesh((0,0),(1,1),(5,3))
	a = 0.1
	inputs = [np.array([0,0]),np.array([1.2*a,a]),np.array([a,-1.5*a]),np.array([-a,a]),np.array([-a,-a])]
	fts = mesh.compute_FTS(inputs)

	init_elem = list(mesh.collision(np.array([0,0]),np.array([0.2,0.2])))
	final_elem = list(mesh.collision(np.array([0.8,0.]),np.array([1.0,1.0])))

	init = random.choice(init_elem)
	fts.graph['initial'] = set(init_elem)

	#fts.show("lkjlkj")

	env = MonotoneEnvironment(mesh)

	labels = {}
	states = mesh.elements + [mesh.out]
	for i,elem in enumerate(states):
		if elem in init_elem:
			env.regions[i] = "i"
		if elem in final_elem:
			env.regions[i] = "f"
	regions = env.regions+[mesh.out]
	nx.set_node_attributes(fts,'label',{elem:set([r]) for elem,r in  zip(states,regions)})
	nx.set_edge_attributes(fts,'weight',{e:1.0 for e in  fts.edges()})

	q1 = Quad("q1","i",env)

	print fts.nodes(data=True)
	print mesh.elements

	#fts.show("model")

	ltl = BA(formula="[]!out && []<>f && []<>i")
	ba = ltl.undeterministic_fts_product(fts)

	ba.minimize()
	#ba.show("product")

	plan = ba.backward_reachability(mesh,inputs, show_dfs=False)

	planner = MonotonePlanner(q1,mesh,plan)

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