import sys
sys.path.append("../core")

from automata import *
from agents import *
from gui import *
from environment import *
from simulator import *

import itertools
import networkx as nx


def generate_paths_async(path,idp,dim,D=1):
	pref = path[:idp+1]
	suf = path[idp+2:]
	paths = []
	N = len(path)
	for p in itertools.product(range(D+1),repeat=dim):
		if sum(p)==0 or sum(p)==dim:
			continue
		prev = tuple(copy.copy([path[(idp+n)%N][i] for i,n in enumerate(p)]))
		next = tuple(copy.copy([path[(idp+n+1)%N][i] for i,n in enumerate(p)]))
		paths.append(pref+[prev,next]+suf)
	return paths

def init():
	env_data = { "rega":rect((0,0),(1,1)),
			"regb":rect((1,3),(2,4)),
			"regc":rect((0,3),(1,4)),
			"regd":rect((3,0),(4,1))}

	env = Environment(env_data)

	q1 = Quad("q1","rega",env)
	q2 = Quad("q2","regb",env)

	ts1 = q1.get_FTS()
	ts2 = q2.get_FTS()

	sync_prod = ts1.fts_product(ts2)
	async_prod = ts1.fts_async_product(ts2)

	ltl = BA(formula="[]<>(eregb) && []<>(erega) && []<>aregc")
	ba = ltl.fts_product(sync_prod)
	async_ba = ltl.fts_product(async_prod)
	print ltl

	pref,pref_cost,suf,suf_cost= ba.dijkstra_plan_networkX(10)


	pr1 = [p[0][0] for p in pref]
	pr2 = [p[0][1] for p in pref]
	su1 = [s[0][0] for s in suf]
	su2 = [s[0][1] for s in suf]

	q1.planner = (pr1,su1)
	q2.planner = (pr2,su2)


	prefix = [p[0] for p in pref]
	suffix = [p[0] for p in suf]
	#print ba.trace_run(prefix,suffix)
	path = prefix + suffix
	n = len(suffix)
	labels1 = nx.get_node_attributes(ts1,'label')
	labels2 = nx.get_node_attributes(ts2,'label')
	print prefix
	print suffix


	suffix_sync = []
	for i in range(n):
		suf = generate_paths_async(suffix,i-1,2,D=2)
		sync = [ba.trace_run(prefix,p) for p in suf]
		suffix_sync.append(not all(sync))

	print suffix_sync
	prefix_sync = [False for p in pr1]

	q1.sync = (prefix_sync,suffix_sync)
	q2.sync = (prefix_sync,suffix_sync)

	trace = []
	t = prefix + suffix
	for s in t:
		l1 = copy.copy(labels1[s[0]])
		l2 = copy.copy(labels2[s[1]])
		L = l1.pop()[-1]+l2.pop()[-1]
		trace.append(L)
	print trace
	print prefix_sync + suffix_sync
	
	sim = Simulator()

	sim.add("q1",q1)
	sim.add("q2",q2)

	sync = SyncPool([q1,q2], prefix_sync,suffix_sync)
	sim.sync_pool.append(sync)

	return sim,env


if __name__ == '__main__':
	sim,env = init()
	gui = GUI(env,sim)
	sim.start()
	gui.start()