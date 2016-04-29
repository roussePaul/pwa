import sys
sys.path.append("../core")

from automata import *
from agents import *
from simulator import *
from gui import *
from environment import *

import itertools
import networkx as nx

def init():
	env_data = { "rega":rect((0,0),(1,1)),
			"regb":rect((1,3),(2,4)),
			"regc":rect((0,3),(1,4)),
			"regd":rect((3,0),(4,1))}

	env = Environment(env_data)

	q1 = Quad("q1","rega",env)

	ts1 = q1.get_FTS()

	ltl = BA(formula="[]<>(regb) && []<>(rega)")
	ba = ltl.fts_product(ts1)

	pref,pref_cost,suf,suf_cost= ba.dijkstra_plan_networkX(10)

	pr1 = [p[0] for p in pref]
	su1 = [s[0] for s in suf]

	q1.planner = (pr1,su1)

	prefix = [p[0] for p in pref]
	suffix = [p[0] for p in suf]
	
	sim = Simulator()

	sim.add("q1",q1)
	
	return sim,env


if __name__ == '__main__':
	sim,env = init()
	gui = GUI(env,sim)
	sim.start()
	gui.start()