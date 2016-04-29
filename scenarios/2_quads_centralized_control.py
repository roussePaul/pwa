import sys
sys.path.append("../core")

from automata import *
from agents import *
from gui import *
from environment import *
from simulator import *
import time
import itertools
import networkx as nx

def init():
	area = np.array(rect([0,0],[1,1]))
	N=10
	points = np.random.uniform(low=0.0,high=1.0,size=(N,2))
	labels = ["reg"+chr(ord('a')+i) for i in range(N)]
	env = DelaunayEnvironment(points,area,labels)

	# area = np.array(rect([0,0],[1,1]))
	# r = 0.1
	# a = 0.5*r
	# b = math.sqrt(3)/2*r
	# X1 = np.arange(0,1,2*b)
	# X2 = np.arange(b,1,2*b)
	# Y1 = np.arange(0,1,2*a)
	# Y2 = np.arange(a,1,2*a)
	# pts1 = np.dstack(np.meshgrid(X1, Y1)).reshape(-1, 2)
	# pts2 = np.dstack(np.meshgrid(X2, Y2)).reshape(-1, 2)
	# pts = np.concatenate((pts1,pts2),axis=0)

	# labels = [str(r) for r in range(pts.shape[0])]
	# print labels
	# env = DelaunayEnvironment(pts,area,labels)

	# env.show()
	# return 
	# env.plot(plt)
	# plt.show()
	# return 

	q1 = Quad("q1","rega",env)
	q2 = Quad("q2","regb",env)

	ts1 = q1.get_FTS()
	ts2 = q2.get_FTS()


	sync_prod = ts1.fts_product(ts2)

	ltl = BA(formula="[]<>(eregb) && []<>(erega) && []<>(eregc) && []<>(eregd)")
	ba = ltl.fts_product(sync_prod)

	t = time.time()
	pref,pref_cost,suf,suf_cost= ba.dijkstra_plan_networkX(10)
	print "Dijkstra in ",time.time()-t

	print len(ltl.nodes())
	print len(ba.nodes())

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