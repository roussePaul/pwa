import sys
sys.path.append("../core")

from automata import *
from agents import *
from gui import *
from environment import *
from simulator import *

import itertools
import networkx as nx
import copy

def init():
	env_data = { "rega":rect((0,0),(1,1)),
			"regb":rect((1,3),(2,4)),
			"regc":rect((0,3),(1,4)),
			"regd":rect((3,0),(4,1))}

	env = Environment(env_data)

	q1 = Quad("q1","rega",env)

	ts1 = q1.get_FTS()
	ts1.show("xsfbvb")
	ts1.graph['type'] = "quad"


	# Build measurement model
	measurement_model = FTS()
	nodes = env.elements
	edges = [(nodes[2],nodes[1]),(nodes[1],nodes[3]),(nodes[3],nodes[2]),(nodes[0],nodes[0])]
	labels = env_data.keys()
	measurement_model.add_nodes_from([(n,{'label':set([])}) for n in env.elements])
	measurement_model.add_edges_from(edges,weight=1.0)
	measurement_model.graph['initial'] = set([nodes[1]])
	measurement_model.graph['symbols'] = set(labels)
	measurement_model.graph['type'] = "meas"
	agent = MeasureProcess(1.0,copy.copy(measurement_model),env)
	agent.notice = lambda : sim.obj_list["q1"].notice_measurement_transition()


	# Build
	prod = ts1.fts_compose(measurement_model)

	# Add observation
	for (n,d) in prod.nodes(data=True):
		if n[0]==n[1]:
			d['label'].add('found')
		else:
			d['label'].add('notfound')

	prod.graph['symbols'].add('found')
	prod.graph['symbols'].add('notfound')

	for n,d in prod.nodes(data=True):
		print n,d

	ltl = BA(formula="[]<>found && [] !regb")
	ba = ltl.fts_product(prod)

	pref,pref_cost,suf,suf_cost= ba.dijkstra_plan_networkX(10)


	pr1 = [p[0][0] for p in pref]
	su1 = [s[0][0] for s in suf]

	print pref
	print suf

	q1.planner = (pr1,su1)

	q1.replan = True
	q1.measurement_model = measurement_model
	q1.prod = prod
	q1.ba = ba
	q1.ts1 = ts1
	q1.ltl = ltl

	q1.no_sync = True
	meas = Measure(lambda sim: "found" if sim.obj_list["q1"].current_region==sim.obj_list["agent"].current_region else "notfound")
	q1.sensors.append(meas)

	pr = [p[0] for p in pref]
	su = [s[0] for s in suf]
	q1.ref_trace = (prod.run_trace(pr),prod.run_trace(su))
	sim = Simulator()

	sim.add("q1",q1)
	sim.add("agent",agent)

	return sim,env


if __name__ == '__main__':
	sim,env = init()
	gui = GUI(env,sim)
	sim.start()
	gui.start()