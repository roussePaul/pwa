import sys
sys.path.append("../core")
import os

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

import pickle

def init(filename="save/save.dat",show_env=False):

	mesh = MonotoneSystem(out_state=True)
	N = 7
	mesh.rectangular_mesh((0,0),(1,1),(N,N))
	a = 0.2
	inputs = [np.array([0,0]),np.array([a,0]),np.array([-a,0]),np.array([0,a]),np.array([0,-a])]
	inputs = [np.array([0,0]),np.array([a,0]),np.array([-a,0]),np.array([0,a]),np.array([0,-a]),np.array([a,a]),np.array([a,-a]),np.array([-a,a]),np.array([-a,-a]),]

	print inputs
	print "Abstraction..."

	b = 0.2
	noise = [np.array([-b,-b]),np.array([b,b])]

	init_elem = list(mesh.collision(np.array([0.0,0.0]),np.array([1,0.3])))
	final_elem = list(mesh.collision(np.array([0.6,0.6]),np.array([1,1])))
	collision = list(mesh.collision(np.array([0.5,0.5]),np.array([0.51,0.51])))

	init = random.choice(init_elem)
 
	print final_elem
	#fts.show("lkjlkj")

	env = MonotoneEnvironment(mesh)
	states = mesh.elements + [mesh.out]
	for i,elem in enumerate(states):
		if elem in init_elem:
			env.regions[i] = "i"
		if elem in final_elem:
			env.regions[i] = "f"
		if elem in collision:
			env.regions[i] = "c"
	regions = env.regions+[mesh.out]

	labels = {elem:set([r]) for elem,r in  zip(states,regions)}

	env.show_reg = ["i","f","c"]
	if show_env:
		env.show()
		return

	fts = mesh.compute_FTS(inputs,noise)
	fts.graph['initial'] = set(init_elem)

	nx.set_node_attributes(fts,'label',labels)
	nx.set_edge_attributes(fts,'weight',{e:1.0 for e in  fts.edges()})


	print "Product..."
	ltl = BA(formula="([]!out) && ([]!c)&& ([]<>f) && ([]<>i) ")
	#ltl.show("ltl")
	ba = ltl.undeterministic_fts_product(fts,verbose=1)
	
	print "Minimize..."
	ba.minimize()

	#ba.show("buchi")
	print ba.graph['accept']

	print "Plan..."
	t = time.time()

	print "Accepted state size", len(ba.nodes())
	plan = ba.backward_reachability(mesh,inputs,show_dfs=False,verbose=0,max_fixed_point_size=4,environment=None,plt=plt)

	if plan == None:
		return

	#plan.show("plan")

	# plan2 = copy.copy(plan)
	# set_node = set([u for e in plan2.edges() for u in e if (e[0] in plan2.graph['initial']) or (e[1] in plan2.graph['initial'])])
	# plan2.remove_nodes_from(set(plan2.nodes()).difference(set_node))
	# plan2.graph['accept'] = set([])
	# plan2.show("dfgdg")
	print "Plan generated in ",time.time()-t,"seconds for ",len(ba.nodes())," states"

	#plan.show("plan")


	init = env.get_elem_in_region("i")
	print init
	init_state = env.get_baricenter(init)
	

	q1 = MonotoneQuad("q1",init_state)


	planner = MonotonePlanner(q1,mesh,plan,verbose=0)

	controller = ControlAutomaton(copy.copy(mesh),copy.copy(plan))
	controller.save("plans/"+os.path.splitext(__file__)[0]+"_plan.p")

	sim = Simulator()

	sim.add("q1",q1)
	sim.add("planner",planner)

	save_file = open(filename,"wb")
	save_list = [env,mesh,fts,planner,env]
	with open(filename, "wb") as f:
		pickle.dump(save_list,save_file)

	return sim,env

if __name__ == '__main__':
	
	command = sys.argv[1:]

	if "show" in command:
		init(show_env=True)


	filename = "save/"+os.path.splitext(__file__)[0]+"_plan.p"
	if "generate" in command:
		sim,env = init(filename=filename)
	elif "product" in command:
		sim,env = init(filename=filename)
	else:
		data = []
		with open(filename, "rb") as f:
			for d in pickle.load(f):
				data.append(d)

		[env_mesh,mesh,fts,planner,env_plot] = data
		env = env_plot

		sim = Simulator()
		sim.add(planner.system.name,planner.system)
		sim.add("planner",planner)

	if "gui" in command:
		gui = GUI(env,sim)
		sim.start_simulation(realtime=True)
		#sim.wait_for_keyboard_interrupt()
		gui.start()
	elif "trace" in command:
		## Trace
		sim.simulate(100.0,show_progress=True)
		env.plot(plt)
		sim.plot_all(plt)
		plt.show()
	elif "controls" in command:
		planner.show_controls(plt,env)