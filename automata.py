import fst
from LTL.buchi import buchi_from_ltl
import networkx as nx
import matplotlib.pyplot as plt
from graphviz import Source
from networkx.classes.digraph import DiGraph
from compiler.ast import flatten
import copy

import numpy as np
import random
import threading
import time
import matplotlib.animation as animation
import itertools
import matplotlib.cm as cmx
import matplotlib.colors as colors
import time
import sys
sys.path.append('LTL/')

from ts import MotionFts, ActionModel, MotActModel
from planner import ltl_planner
from buchi import mission_to_buchi

import networkx as nx
from networkx.drawing.nx_agraph import to_agraph

from buchi import check_label_for_buchi_edge
from discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal, improve_plan_given_history
from networkx import dijkstra_predecessor_and_distance

def compute_path_from_pre(pre, target):
	#print 'pre: %s with size %i' %(pre, len(pre))
	n = target
	path = [n]
	while n in pre:
		#print 'before append'
		#print 'now at node %s' %str(n)
		pn_list = pre[n]
		#print 'its pre_list %s' %str(pn_list)
		if not pn_list:
			break
		pn = pn_list[0]
		#print '[0] of pn_list %s' %str(pn)
		path.append(pn)
		#print 'path: %s' %path
		n = pn
	path.reverse()
	return path


def to_list(x):
	if type(x)==list:
		return x


class FTS(DiGraph):
	def __init__(self, symbols=set([]),initial=set([])):
		DiGraph.__init__(self, symbols=symbols, type="fts", initial=initial)

		self.elem = 1

	def fts_product(self,fts2):
		assert type(fts2) == FTS
		result =  FTS(set([]),set([])) 
		
		nodes_prod = lambda u,v: tuple((list(u) if self.elem>1 else [u]) + (list(v) if fts2.elem>1 else [v]))
		labels_prod = lambda u,v: flatten((u,v))

		labels = nx.get_node_attributes(self,'label')
		labels_fts2 = nx.get_node_attributes(fts2,'label')

		for u in self.nodes():
			for v in fts2.nodes():
				result.add_node(nodes_prod(u,v),label=labels_prod(labels[u],labels_fts2[v]),weight=1.0)
		for (u,v) in self.edges():
			for (x,y) in fts2.edges():
				result.add_edge(nodes_prod(u,x),nodes_prod(v,y),weight=1.0)

		for u in self.graph['initial']:
			for v in fts2.graph['initial']:
				result.graph['initial'].add(nodes_prod(u,v))
		
		for u in self.graph['symbols']:
			for v in fts2.graph['symbols']:
				result.graph['symbols'].add(nodes_prod(u,v))

		result.elem = self.elem + fts2.elem

		return copy.deepcopy(result)

    def composition(self, reg, act):
        prod_node = (reg, act)
        if not self.has_node(prod_node):
            new_label = self.graph['region'].node[reg]['label'].union(self.graph['action'].action[act][2])
            self.add_node(prod_node, label=new_label, region=reg, action=act, marker='unvisited')
            if ((reg in self.graph['region'].graph['initial']) and (act == 'None')):
                self.graph['initial'].add(prod_node)
        return prod_node

	def fts_compose(self,fts2):
		

    def build_initial(self):
        for reg_init in self.graph['region'].graph['initial']:
            init_prod_node = self.composition(reg_init, 'None')

    def build_full(self):
        for reg in self.graph['region'].nodes_iter():
            for act in self.graph['action'].action.iterkeys():
                prod_node = self.composition(reg, act)
                # actions 
                label = self.graph['region'].node[reg]['label']
                for act_to in self.graph['action'].allowed_actions(label):
                    prod_node_to = self.composition(reg, act_to)
                    self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label= act_to, marker= 'visited')
                # motions
                for reg_to in self.graph['region'].successors_iter(reg):
                    if reg_to != reg:
                        prod_node_to = self.composition(reg_to, 'None')
                        self.add_edge(prod_node, prod_node_to, weight=self.graph['region'][reg][reg_to]['weight'], label= 'goto', marker= 'visited')

	def show(self):
		A=to_agraph(self)
		src = Source(A)
		src.format = 'png'
		src.render("test-output/graph",view=True)

	def minimize(self):
		for n in self.nodes():
			path_exist = False
			for m in self.graph['initial']:
				if nx.has_path(self,m,n):
					path_exist = True
					break
			if path_exist==False:
				self.remove_node(n)
			
class BA(DiGraph):
	def __init__(self,formula="",alpha=100,_type="hard_buchi"):
		if formula == "":
			DiGraph.__init__(self,type=_type, initial=set([]), accept=set([]), symbols=set([]))
		else:
			G = buchi_from_ltl(formula,_type)
			DiGraph.__init__(self,G)
		self.alpha = alpha
		self.type = _type

	def fts_product(self,fts):
		result = BA()

		for f_ts_node in fts.nodes_iter():
			for f_buchi_node in self.nodes_iter():
				f_prod_node = self.composition(result,fts,f_ts_node, f_buchi_node)
				for t_ts_node in fts.successors_iter(f_ts_node):
					for t_buchi_node in self.successors_iter(f_buchi_node):
							t_prod_node = self.composition(result,fts,t_ts_node, t_buchi_node)
							label = fts.node[f_ts_node]['label']
							cost = fts[f_ts_node][t_ts_node]['weight']
							truth, dist = check_label_for_buchi_edge(self, label, f_buchi_node, t_buchi_node)
							total_weight = cost + self.alpha*dist
							if truth:
								result.add_edge(f_prod_node, t_prod_node, weight=total_weight)
		return result

	def composition(self, result, fts, ts_node, buchi_node):
		prod_node = (ts_node, buchi_node)
		if not self.has_node(prod_node):
			result.add_node(prod_node, ts=ts_node, buchi=buchi_node, marker='unvisited')
			if ((ts_node in fts.graph['initial']) and
				(buchi_node in self.graph['initial'])):
				result.graph['initial'].add(prod_node)
			if (buchi_node in self.graph['accept']):
				result.graph['accept'].add(prod_node)
		return prod_node

	def dijkstra_plan_networkX(self, beta=10):
		# requires a full construct of product automaton
		start = time.time()
		runs = {}
		loop = {}
		cycle = {}
		line = {}
		# minimal circles
		for prod_target in self.graph['accept']:
			loop_pre, loop_dist = dijkstra_predecessor_and_distance(self, prod_target)
			for target_pred in self.predecessors_iter(prod_target):
				if target_pred in loop_dist:
					cycle[target_pred] = self.edge[target_pred][prod_target]["weight"] + loop_dist[target_pred]
			if cycle:
				opti_pred = min(cycle, key = cycle.get)
				suffix = compute_path_from_pre(loop_pre, opti_pred)
				loop[prod_target] = (cycle[opti_pred], suffix)
		# shortest line
		for prod_init in self.graph['initial']:
			line_pre, line_dist = dijkstra_predecessor_and_distance(self, prod_init)
			for target in loop.iterkeys():
				if target in line_dist:
					line[target] = line_dist[target]+beta*loop[target][0]
			if line:
				opti_targ = min(line, key = line.get)
				prefix = compute_path_from_pre(line_pre, opti_targ)
				precost = line_dist[opti_targ]
				runs[(prod_init, opti_targ)] = (prefix, precost, loop[opti_targ][1], loop[opti_targ][0])
		# best combination
		if runs:
			prefix, precost, suffix, sufcost = min(runs.values(), key = lambda p: p[1] + beta*p[3])
			#print '\n==================\n'
			return prefix, precost, suffix, sufcost
			#print '\n==================\n'
		print 'no accepting run found in optimal planning!'

	def show(self):
		A=to_agraph(self)
		for n in self.graph['accept']:
			node = A.get_node(n)
			node.attr['peripheries'] = 2
			
		for n in self.graph['initial']:
			node = A.get_node(n)
			node.attr['penwidth'] = 2
		src = Source(A)
		src.format = 'png'
		src.render("test-output/graph",view=True)


def sat(x,m):
	return x if abs(x)<m else x/abs(x)*m

class Quad:
	def __init__(self,name,init_region,env):
		self.name = name
		self.init_region = init_region
		self.controller = Controller(env)
		self.env = env
		elem = self.env.get_elem_in_region(self.init_region)
		self.controller.goto(elem)
		self.state = self.env.get_baricenter(elem)


		self.current_action = None
		self.planner = None
		self.planner_state = 0

	def sim(self,x,dt):
		if self.planner:
			self.exec_planner()
		u = self.controller.update(x,dt)
		self.state = self.state + u*dt

	def get_FTS(self):
		return self.controller.get_FTS()

	def do(self,elem):
		if self.current_action == elem:
			return self.is_done(elem)
		else:
			self.current_action = elem
			if elem in self.env.elements:
				self.controller.goto(elem)

	def is_done(self,elem):
		if elem in self.env.elements:
			return self.controller.is_done(self.state)
		else:
			return True

	def exec_planner(self):
		if self.is_done(self.current_action):
			self.planner_state += 1
			if self.planner_state<len(self.planner[0]):
				self.current_action = self.planner[0][self.planner_state]

			else:
				ida = (self.planner_state-len(self.planner[0]))%len(self.planner[1])
				self.current_action = self.planner[1][ida]
			if self.current_action in self.env.elements:
				self.controller.goto(self.current_action)
			print self.planner_state,self.current_action



class Simulator(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.sim_list = []
		self.stop = False

	def add(self,obj):
		self.sim_list.append(obj)

	def sim(self,dt):
		for obj in self.sim_list:
			meas = obj.state
			obj.sim(meas,dt)

	def init_objects(self):
		for obj in self.sim_list:
			obj.init_pos()

	def run(self):
		while self.stop == False:
			self.sim(0.01)
			time.sleep(0.01)

	def get_obj_pos(self):
		return [o.state for o in self.sim_list]

class Controller:
	def __init__(self,env):
		self.speed = 5.0
		self.env = env
		self.ref_point = np.array([1,1])
		self.ref_elem = random.choice(self.env.elements)
		self.goto(self.ref_elem)

	def goto(self,to_elem):
		self.ref_elem = to_elem
		self.ref_point = self.env.get_baricenter(to_elem)

	def is_done(self,x):
		return np.linalg.norm(self.ref_point-x)<0.2

	def update(self,x,dt):
		d = np.linalg.norm(self.ref_point-x)
		if d<1e-6:
			return x*0
		n = (self.ref_point-x)/d
		return sat(d,1)*self.speed*n

	def get_FTS(self):
		nodes = [(e,{'label':set([r]),'weight':1.0}) for e,r in zip(self.env.elements,self.env.regions)]
		L = self.env.elements
		E = list(itertools.permutations(L,2))
		edges = [(u,v,{'weight':1.0}) for u,v in itertools.permutations(L,2)]

		symbols = set(self.env.regions)
		fts = FTS(symbols,set([self.ref_elem]))

		fts.add_nodes_from(nodes)
		fts.add_edges_from(edges)
		return fts



def get_cmap(N):
	'''Returns a function that maps each index in 0, 1, ... N-1 to a distinct 
	RGB color.'''
	color_norm  = colors.Normalize(vmin=0, vmax=N-1)
	scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='jet') 
	def map_index_to_rgb_color(index):
		return scalar_map.to_rgba(index)
	return map_index_to_rgb_color

class Environment:
	#vectrices: list of points
	#elements: [(v1,v2,v3),(v2,v3,...),...] -> must be convex
	#regions: ["l1","l1","l3"]

	def __init__(self,env):
		self.vectrices = []
		self.elements = []
		self.regions = []
		for l,e in env.items():
			elem = []
			for n in e:
				eq = [np.array_equal(x,n) for x in self.vectrices ]
				if True not in eq:
					idn = len(self.vectrices)
					self.vectrices.append(n)
				else:
					idn = eq.index(True) 
				elem.append(idn)
			self.elements.append(tuple(elem))
			self.regions.append(l)

		self.colors = get_cmap(len(set(self.regions)))

	def get_point_in_region(self,region):
		elem = [e for e,r in zip(self.elements,self.regions) if r==region]
		if len(elem)>0:
			return self.get_baricenter(random.choice(elem))
		else:
			print "Region not found"

	def get_elem_in_region(self,region):
		elem = [e for e,r in zip(self.elements,self.regions) if r==region]
		if len(elem)>0:
			return random.choice(elem)
		else:
			print "Region not found"

	def get_baricenter(self,elem):
		x = np.array([0.0,0.0])
		for n in elem:
			x += self.vectrices[n]
		return x / len(elem)


	def plot(self):
		v = np.array(self.vectrices)
		elements = 	np.array([list(e) for e in self.elements])
		region_colors = list(itertools.chain.from_iterable([[r]*len(e) for r,e in zip(self.regions,self.elements)]))
		regions = {c:i for i,c in enumerate(set(region_colors))}
		ax = plt.gca()
		for e,r in zip(self.elements,self.regions):
			pts = [list(self.vectrices[n]) for n in e]
			if r=="obstacle":
				print r
				ax.add_patch(plt.Polygon(pts, closed=True,fill=False,hatch='//'))
			else:
				ax.add_patch(plt.Polygon(pts, closed=True,fill=True,color=self.colors(regions[r])))

		for r in set(self.regions):
			p = self.get_point_in_region(r)
			ax.text(p[0], p[1], r, fontsize=15,horizontalalignment='center',verticalalignment='center',bbox={'facecolor':'white', 'alpha':0.5, 'pad':1, 'edgecolor':'none'})


		return ax


class GUI:
	def __init__(self,env,sim):
		self.env = env
		self.sim = sim
		self.rate = 25

		self.fig = plt.figure()

		self.ax_env = env.plot()
		self.ax_obj, = plt.plot([0],[0],"ro")


		self.anim = animation.FuncAnimation(self.fig, self.redraw, 25, fargs=None,interval=50, blit=False)

	def redraw(self,d):
		p = np.array(self.sim.get_obj_pos())

		try:
			self.ax_obj.set_data(p[:,0],p[:,1])
		except IndexError,e:
			print e,p
		return self.ax_obj,

	def start(self):
		plt.show()
		self.sim.stop = True



rect = lambda a,b: [(a[0],a[1]),(a[0],b[1]),(b[0],b[1]),(b[0],a[1])]



def init():
	env_data = { "start":rect((0,0),(1,1)),
			"upload":rect((3,3),(4,4)),
			"obstacle":rect((2,2),(2.5,2.5)),
			"measure":rect((0,3),(1,4))}

	env = Environment(env_data)

	q1 = Quad("q1","start",env)
	q2 = Quad("q2","upload",env)


	ts = q1.get_FTS()
	action_dict={
				 'actupload': (100, 'upload', set(['actupload'])),
				 'actdownload': (60, 'measure', set(['actdownload']))
				}
	action = ActionModel(action_dict)
	model = MotActModel(ts,action)

	planner  = ltl_planner(model,'[]<>start && []<>upload',None)


	# buchi = mission_to_buchi('([]<>actdownload) && ([](actdownload -> X(!actdownload U actupload)))',None)
	# p = {e:env.get_baricenter(e) for e in env.elements}
	# l = {e:r for e,r in zip(env.elements,env.regions)}
	# nx.draw(ts,pos=p,labels=l)
	# plt.show()
	STYLE = 'static'
	planner.optimal(10, STYLE)
	print_graph(model)

	planner.run.pre_plan
	prefix = [n for n in planner.run.pre_plan]
	suffix = [n for n in planner.run.suf_plan]
	q1.planner = (prefix,suffix)

	sim = Simulator()

	sim.add(q1)
	sim.add(q2)

	return sim,env


def print_graph(ts):
	print ">>>>>>>>>>>>>>>>>>>>>>>>>"

	for n in ts.nodes(data=True):
		print n

	print "<<<<<<<<<<<<<<<<<<<<<<<<<"


if __name__ == '__main__':
	env_data = { "start":rect((0,0),(1,1)),
			"upload":rect((3,3),(4,4)),
			"kjh":rect((0,3),(1,4))}

	env = Environment(env_data)

	q1 = Quad("q1","start",env)
	q2 = Quad("q2","upload",env)


	ts1 = q1.get_FTS()
	ltl = BA(formula="[]<>start && []<>upload")
	ba = ltl.fts_product(ts1)
	print ba.dijkstra_plan_networkX(10)

	ba.show()
