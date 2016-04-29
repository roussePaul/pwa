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
	mesh.rectangular_mesh((0,0),(1,1),(5,5))
	a = 0.1
	inputs = [np.array([0,0]),np.array([0,a]),np.array([a,0]),np.array([-a,0]),np.array([0,-a])]
	fts = mesh.compute_FTS(inputs)

	init_elem = list(mesh.collision(np.array([0,0]),np.array([0.2,0.2])))
	avoid_elem = list(mesh.collision(np.array([0.4,0.4]),np.array([0.6,0.6])))
	final_elem = list(mesh.collision(np.array([0.8,0.8]),np.array([1.0,1.0])))

	init = random.choice(init_elem)
	fts.graph['initial'] = set(init_elem)

	#fts.show("lkjlkj")

	env = MonotoneEnvironment(mesh)

	labels = {}
	for i,elem in enumerate(mesh.elements):
		if elem in init_elem:
			env.regions[i] = "i"
		if elem in avoid_elem:
			env.regions[i] = "c"
		if elem in final_elem:
			env.regions[i] = "f"
	nx.set_node_attributes(fts,'label',{elem:set([r]) for elem,r in  zip(mesh.elements,env.regions)})
	nx.set_edge_attributes(fts,'weight',{e:1.0 for e in  fts.edges()})


	q1 = Quad("q1","i",env)

	#fts.show("lkjlkj")

	ltl = BA(formula="[]!c && []<>f && []<>i")
	ba = ltl.undeterministic_fts_product(fts)

	ba.minimize()
	#ba.show("wefuysdg")

	# S,suppressed_nodes,suppressed_edges = ba.get_safe_states()

	# print "Suppressed nodes", suppressed_nodes
	# print "Suppressed edges", list(suppressed_edges)

	# #ba.show("klj")
	# labels = nx.get_edge_attributes(ba,'label')
	# edges = [(u[0],k)  for u,l in labels.iteritems() for k in list(l) if (u[0][0],k) in suppressed_edges]
	# edges_to_remove = []
	# for n,l in edges:
	# 	for v in ba.neighbors(n):
	# 		if l in ba[n][v]['label']:
	# 			print n,v,l, ba[n][v]['label'],ba[n][v]['control']
	# 			ba[n][v]['label'].remove(l)
	# 			ba[n][v]['control'].pop(l)
	# 			if len(ba[n][v]['label'])==0:
	# 				edges_to_remove.append((n,v))

	# ba.remove_edges_from(edges_to_remove)

	# ba.minimize()
	# ba.graph['accept'] = ba.graph['accept'].intersection(set(ba.nodes()))
	# ba.graph['initial'] = ba.graph['initial'].intersection(set(ba.nodes()))

	#ba.show("dgfjhfgjdfhv")


	fts = ba.plan(mesh)
	fts.minimize()
	fts.show("lkjlkj")


	# fts,plan = ba.get_fts_plan()
	
	#fts.show("kljkljh")

	planner = MonotonePlanner(q1,mesh,fts)
	# G = nx.DiGraph()
	# G.add_nodes_from(ba.nodes())
	# G.add_edges_from(ba.edges())

	# #cycles =  sorted(nx.simple_cycles(G),key=lambda l:len(l))

	# print "CYCLE"+"\n"*3
	# ba.create_fair_graph(mesh)

	# print "\n"*3 + "END"

	# control = nx.get_node_attributes(plan,"apply_control")
	# label = nx.get_edge_attributes(plan,"label")
	# for n in plan.nodes():
	# 	c = set([])
	# 	for v in plan.neighbors(n):
	# 		c = label[(n,v)].union(c)
	# 	c = c.difference(set([str(control[n])]))
	# 	for unused_control in c:
	# 		plan.remove_labeled_edge(n,unused_control)
	# plan.show("jshdgfhsjdf")
	# print control
	# return 
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

