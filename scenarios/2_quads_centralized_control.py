import sys
sys.path.append("../core")

from automata import *
from agents import *
from gui import *
from environment import *
from simulator import *

import itertools
import networkx as nx

def init():
	area = np.array(rect([0,0],[1,1]))
	N=5
	points = np.random.uniform(low=0.0,high=1.0,size=(N,2))
	labels = ["reg"+chr(ord('a')+i) for i in range(N)]
	env = DelaunayEnvironment(points,area,labels)

	# env.plot(plt)
	# plt.show()
	# return 

	q1 = Quad("q1","rega",env)
	q2 = Quad("q2","regb",env)

	ts1 = q1.get_FTS()
	ts2 = q2.get_FTS()


	ts1.show("lkjlj")
	sync_prod = ts1.fts_product(ts2)

	ltl = BA(formula="[]<>(eregb) && []<>(erega) && []<>(eregc) && []<>(eregd)")
	ba = ltl.fts_product(sync_prod)

	pref,pref_cost,suf,suf_cost= ba.dijkstra_plan_networkX(10)


	prefix = [p[0] for p in pref]
	suffix = [p[0] for p in suf]

	sim = Simulator()

	sim.add("q1",q1)
	sim.add("q2",q2)

	p = [{"q1":u,"q2":v} for u,v in prefix]
	s = [{"q1":u,"q2":v} for u,v in suffix]
	centralized_planner = CentralizedPlanner([q1,q2],(p,s))

	sim.add("planner",centralized_planner)

	return sim,env

if __name__ == '__main__':
	sim,env = init()
	gui = GUI(env,sim)
	sim.start()
	gui.start()