import sys
sys.path.append("../core")

from automata import *
from agents import *
from gui import *
from environment import *
from simulator import *
from monotone_system import *
from plan_control import *

import time
import itertools
import networkx as nx
import random
import copy

from matplotlib import pyplot as plt

def init():
	mesh = MonotoneSystem(out_state=True)
	N = 12
	mesh.rectangular_mesh((0,0),(1,1),(N,N))
	a = 0.1
	#inputs = [np.array([0,0]),np.array([a,0]),np.array([-a,0]),np.array([0,a]),np.array([0,-a])]
	#inputs = [np.array([0,0]),np.array([a,a]),np.array([a,-a]),np.array([-a,a]),np.array([-a,-a])]
	inputs = [np.array([0,0]),np.array([a,0]),np.array([math.cos(2*math.pi/3)*a,math.sin(2*math.pi/3)*a]),np.array([math.cos(-2*math.pi/3)*a,math.sin(-2*math.pi/3)*a])]
	print "Abstraction..."
	fts = mesh.compute_FTS(inputs)

	init_elem = list(mesh.collision(np.array([0,0]),np.array([0.1,0.1])))
	final_elem = list(mesh.collision(np.array([0.8,0.8]),np.array([1.0,1.0])))
	collision = list(mesh.collision(np.array([0.65,0.65]),np.array([0.7,0.7])))
	goal = list(mesh.collision(np.array([0.0,0.8]),np.array([0.2,1.0])))

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
		if elem in collision:
			env.regions[i] = "c"
	regions = env.regions+[mesh.out]
	nx.set_node_attributes(fts,'label',{elem:set([r]) for elem,r in  zip(states,regions)})
	nx.set_edge_attributes(fts,'weight',{e:1.0 for e in  fts.edges()})

	env.show_reg = ["i","f","c"]

	q1 = Quad("q1","i",env)


	print "Product..."
	ltl = BA(formula="([]!out)  &&([]!c) && ([]<>f) && ([]<>i)")
	ba = ltl.undeterministic_fts_product(fts)
	
	#ba.show("buchi")

	print "Minimize..."
	ba.minimize()

	print "Plan..."
	t = time.time()
	plan = ba.backward_reachability(mesh,inputs,show_dfs=False,verbose=0)

	# plan2 = copy.copy(plan)
	# set_node = set([u for e in plan2.edges() for u in e if (e[0] in plan2.graph['initial']) or (e[1] in plan2.graph['initial'])])
	# plan2.remove_nodes_from(set(plan2.nodes()).difference(set_node))
	# plan2.graph['accept'] = set([])
	# plan2.show("dfgdg")
	print "Plan generated in ",time.time()-t,"seconds for ",len(ba.nodes())," states"

	#plan.show("plan")

	planner = MonotonePlanner(q1,mesh,plan,verbose=0)

	controller = ControlAutomaton(copy.copy(mesh),copy.copy(plan))
	controller.save("plans/plan.p")

	sim = Simulator()

	sim.add("q1",q1)
	sim.add("q1_planner",planner)

	return sim,env

if __name__ == '__main__':
	sim,env = init() 

	if False:
		gui = GUI(env,sim)
		sim.start_simulation(realtime=True)
		#sim.wait_for_keyboard_interrupt()
		gui.start()
	elif True:
		## Trace
		sim.simulate(30.0,show_progress=True)
		env.plot(plt)
		sim.plot_all(plt)
		plt.show()
