import numpy as np
import random
import threading
import time
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import itertools
import matplotlib.cm as cmx
import matplotlib.colors as colors

import sys
sys.path.append('LTL/')

from ts import MotionFts, ActionModel, MotActModel
from planner import ltl_planner
from buchi import mission_to_buchi

import networkx as nx

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
		nodes = {e:set([r]) for e,r in zip(self.env.elements,self.env.regions)}
		L = nodes.keys()
		edges = list(itertools.permutations(L,2))

		symbols = list(set(self.env.regions))
		F = MotionFts(nodes,symbols,'sml')
		F.set_initial(self.ref_elem)

		F.add_un_edges(edges)
		return F



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
	sim,env = init()
	gui = GUI(env,sim)

	sim.start()

	gui.start()