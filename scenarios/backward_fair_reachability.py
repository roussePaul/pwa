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

from matplotlib import pyplot as plt

def flatten(l):
    for el in l:
        if isinstance(el, list) and not isinstance(el, basestring):
            for sub in flatten(el):
                yield sub
        else:
            yield el

def init():
	mesh = MonotoneSystem(out_state=True)
	mesh.rectangular_mesh((0,0),(1,1),(7,7))
	a = 0.1
	inputs = [np.array([0,0]),np.array([a,a]),np.array([a,-a]),np.array([-a,a]),np.array([-a,-a])]
	inputs = [np.array([0,0]),np.array([a,0]),np.array([math.cos(2*math.pi/3)*a,math.sin(2*math.pi/3)*a]),np.array([math.cos(-2*math.pi/3)*a,math.sin(-2*math.pi/3)*a])]
	fts = mesh.compute_FTS(inputs)

	init_elem = list(mesh.collision(np.array([0,0]),np.array([0.2,0.2])))
	final_elem = list(mesh.collision(np.array([0.8,0.8]),np.array([1.0,1.0])))
	collision = list(mesh.collision(np.array([0.4,0.4]),np.array([0.5,0.5])))
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

	q1 = Quad("q1","i",env)

	print fts.nodes(data=True)
	print mesh.elements

	#fts.show("model")

	ltl = BA(formula="([]!out) && ([]!c) && ([]<>f) && ([]<>i)")
	ba = ltl.undeterministic_fts_product(fts)
	ltl.show("buchi")
	ba.minimize()
	# for e in nx.get_node_attributes(ba,'ts').items():
	# 	print e
	# return
	accepted_set = ba.graph['accept']

	generate_all_predecessor_set = lambda node_set: [{u:l} for u,v,d in ba.in_edges(data=True) if u not in node_set and v in node_set for l in d['label']]


	initial_state_found = False
	accepted_set_circled = False

	max_fixed_point_size = 4

	control_dict = {str(u):u for u in inputs}

	initial_control_set = set([(a,None) for a in accepted_set])
	initial_accepted_set = accepted_set
	initial_fixed_point_iterator = DijkstraFixedPoint(ba,generate_all_predecessor_set(initial_accepted_set),initial_accepted_set)

	to_visit = collections.deque([(initial_control_set,initial_fixed_point_iterator)])

	while to_visit and not (initial_state_found and accepted_set_circled):		
		e = to_visit[-1]

		accepted_control,fixed_point_iterator = e
		accepted_set = set([x[0] for x in accepted_control])

		print " "*len(to_visit) + "| iter "+str(len(to_visit))


		found,nodes = fixed_point_iterator.next_fixed_point(max_fixed_point_size)
		
		if not found:
			if len(to_visit)>1:
				to_visit.pop()
			else:
				print "No solution found"
				return

		if found:
			outgoing_edges = [(u,v) for u,l in nodes.items() for v in ba.get_labelled_successors(u)[l] if v in accepted_set]
			need_fairness = not all([u[0]==v[0] for u,v in outgoing_edges])
			print outgoing_edges,need_fairness

			controls = [control_dict[l] for l in nodes.values()]

			sG = nx.DiGraph()
			sG.add_edges_from(ba.subgraph(nodes.keys()).edges())
			try:
				nx.set_node_attributes(sG,'control',{n:control_dict[l] for n,l in nodes.items() if n in sG.nodes()})
			except KeyError:
				print sG.edges()
				print nodes.keys()
				raise KeyError
			print sG.edges(data=True)
			#fair_control = mesh.is_loop_fair(controls)
			fair_control = mesh.is_graph_fair(sG)

			if fair_control or need_fairness==False:
				print "need_fairness",need_fairness,"fair_control",fair_control,[str(c) for c in controls]
				print "Add fair loop",nodes
				X = accepted_control.union(set(nodes.items()))
				node_set = [x[0] for x in X]
				Y =  generate_all_predecessor_set(node_set)
				it = DijkstraFixedPoint(ba,generate_all_predecessor_set(node_set),node_set)

				to_visit.append((X,it))

			else:
				print "Unfair loop",controls

			if ba.graph['initial'].issubset(set(nodes.keys()).union(accepted_set)):
				initial_state_found = True
				print "Path to initial_set found!!!!"
			new_set = accepted_set.union(set(nodes.keys()))
			cycle_accept_set = [(u,l) for u in ba.graph['accept'] for l,succ in ba.get_labelled_successors(u).items() if set(succ).issubset(new_set)]
			if cycle_accept_set and ba.graph['initial'].issubset(set(nodes.keys()).union(accepted_set)):
				soluce = accepted_control.union(set(nodes.items()))
				break

			# c = {n:"red" for n in accepted_set}
			# c.update({n:"green" for n in nodes})
			# ec = {e:"red" for e in [(u,v) for u,l in accepted_control if l!=None for v in ba.get_labelled_successors(u)[l]]}
			# ec.update( {e:"blue" for e in [(u,v) for u,l in nodes.items() if l!=None for v in ba.get_labelled_successors(u)[l]]} )
			# print ec
			# ba.show("jhgfs",colors=c,edges_color=ec)
			# raw_input()

	cycle_accept_set = [(u,l) for u in ba.graph['accept'] for l,succ in ba.get_labelled_successors(u).items() if set(succ).issubset(new_set)]
	print cycle_accept_set
	solution = {u:l for u,l in soluce}
	for u,l in cycle_accept_set:
		solution[u] = l
	print solution

	plan = copy.copy(ba)
	for n,l in solution.items():
		plan.remove_labeled_edge_except(n,l)
	plan.remove_ambiguites()
	plan.minimize()
	# plan.show("plan2")
	# return 
	print ba.graph['initial']

	nx.set_node_attributes(plan,"apply_control",None)
	control = nx.get_node_attributes(plan,"apply_control")
	for n in plan.nodes():
		c = list(plan.get_successor_labels(n))
		if len(c)==0:
			print "Control not found:", n, s
		else:
			s = c[0]
			if len(c)!=1:
				print "Too many controls",n,c
				s = c[0] 
		control[n] = control_dict[s]

	nx.set_node_attributes(plan,"apply_control",control)

	planner = MonotonePlanner(q1,mesh,plan)

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
	else:
		## Trace
		sim.simulate(30.0)
		env.plot(plt)
		sim.plot_all(plt)
		plt.show()

	print sim.obj_list["q1"].trace