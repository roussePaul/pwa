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

	accepted_set = ba.graph['accept']

	generate_predecessor_set = lambda controls,node_set: [(controls,{u:l}) for u,v,d in ba.in_edges(data=True) if u not in node_set and v in node_set for l in d['label']]

	to_visit = collections.deque(generate_predecessor_set(set([(a,None) for a in accepted_set]),accepted_set))

	initial_state_found = False
	accepted_set_circled = False

	visited_states = []

	control_dict = {str(u):u for u in inputs}

	while to_visit and not (initial_state_found and accepted_set_circled):		
		e = to_visit.pop()

		accepted_control,initial_set = e
		accepted_set = set([x[0] for x in accepted_control])

		visited_states.append(e)
		print "\n||||||||||||||||||||||||"
		print initial_set
		print accepted_set

		it = DijkstraFixedPoint(ba,[initial_set],accepted_set)

		for done,nodes in  it:
			print done	
			if done:
				next_it = False

				outgoing_edges = [(u,v) for u,l in nodes.items() for v in ba.get_labelled_successors(u)[l] if v in accepted_set]
				need_fairness = not all([u[0]==v[0] for u,v in outgoing_edges])
				print outgoing_edges,need_fairness

				controls = [control_dict[l] for l in nodes.values()]

				sG = nx.DiGraph()
				sG.add_edges_from(ba.subgraph(nodes.keys()).edges())
				nx.set_node_attributes(sG,'control',{n:control_dict[l] for n,l in nodes.items()})
				print sG.edges(data=True)
				#fair_control = mesh.is_loop_fair(controls)
				fair_control = mesh.is_graph_fair(sG)

				if fair_control or need_fairness==False:
					print "need_fairness",need_fairness,"fair_control",fair_control,[str(c) for c in controls]
					print "Add fair loop",nodes
					X = accepted_control.union(set(nodes.items()))
					Y =  generate_predecessor_set(X,[x[0] for x in X])

					not_visited_states = []
					for y in Y:
						if y not in visited_states:
								not_visited_states.append(y)

					to_visit.extend(not_visited_states)

					next_it = True
				else:
					print "Unfair loop",controls

				if set(nodes.keys()).intersection(ba.graph['initial']):
					initial_state_found = True
					print "Path to initial_set found!!!!"
				new_set = accepted_set.union(set(nodes.keys()))
				cycle_accept_set = [(u,l) for u in ba.graph['accept'] for l,succ in ba.get_labelled_successors(u).items() if set(succ).issubset(new_set)]
				if cycle_accept_set:
					accepted_set_circled = True

				c = {n:"red" for n in accepted_set}
				c.update({n:"green" for n in nodes})
				ba.show("jhgfs",colors=c)
				raw_input()
				if next_it:
					break

	cycle_accept_set = [(u,l) for u in ba.graph['accept'] for l,succ in ba.get_labelled_successors(u).items() if set(succ).issubset(new_set)]
	print cycle_accept_set
	solution = {u:l for u,l in accepted_control}
	for u,l in cycle_accept_set:
		solution[u] = l
	print solution

	plan = copy.copy(ba)
	for n,l in solution.items():
		plan.remove_labeled_edge_except(n,l)
	plan.remove_ambiguites()
	plan.show("kjhkjh")

	return 
	nx.set_node_attributes(plan,"apply_control",None)
	control = nx.get_node_attributes(plan,"apply_control")
	for n in plan.nodes():
		c = list(plan.get_successor_labels(n))
		s = c[0]
		if len(c)!=1:
			print "Too many controls",n,c
			s = c[1] 
		control[n] = control_dict[s]


	nx.set_node_attributes(plan,"apply_control",control)

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