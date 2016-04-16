import threading
import time

from automata import *
from agents import *
from gui import *
from environment import *
from abc import ABCMeta, abstractmethod

class Simulator(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.obj_list = {}
		self.stop = False

		self.get_class_obj = lambda c: [o for o in self.obj_list.values() if c in o.__class__.__mro__]

		self.sync_pool = []
		self.time = 0.0

	def add(self,name,obj):
		self.obj_list[name] = obj

	def sim(self,dt):
		self.time += dt
		for obj in self.get_class_obj(InteractiveAgent):
			obj.update_measure(self)
		for obj in self.get_class_obj(InteractiveAgent):
			obj.update_pos(self)
		for obj in self.get_class_obj(ContinuousSimObj):
			obj.cont_sim(dt)
		for obj in self.get_class_obj(DiscreteSimObj):
			if obj.transition_available(self):
				if not self.wait_for_sync(obj):
					obj.di_sim(self)
		for sp in self.sync_pool:
			i = sp.get_id_sync_done()
			sp.reset_sync(i)
			if i!=None:
				print "SYNC pool ", [obj.name for obj in sp.obj_pool]
				for obj in sp.obj_pool:
					obj.di_sim(self)
		for obj in self.get_class_obj(InteractiveAgent):
			obj.update_measure(self)

		for obj in self.get_class_obj(DiscreteSimObj):
			obj.manage_notification(self)

	def wait_for_sync(self,obj):
		for sp in self.sync_pool:
			if sp.already_noticed(obj,obj.planner_state):
				return True

			if obj in sp.obj_pool and sp.sync_available(obj,obj.planner_state):
				sp.ready(obj,obj.planner_state)
				return sp.is_sync_done(obj.planner_state)
		return False

	def init_objects(self):
		for obj in self.get_class_obj(ContinuousSimObj):
			obj.init_pos()

	def run(self):
		while self.stop == False:
			self.sim(0.01)
			time.sleep(0.01)

# Always simulated
class ContinuousSimObj:
	def __init__(self):
		self.state = np.array([0,0])
		self.u = np.array([0,0])

		self.current_region = None

	def cont_sim(self,dt):
		self.state += self.u*dt
		self.env.get_region(self.state)


def sat(x,m):
	return x if abs(x)<m else x/abs(x)*m

class ControlledSimObj(ContinuousSimObj):
	def __init__(self,env):
		ContinuousSimObj.__init__(self)
		self.speed = 3.0
		self.env = env
		self.ref_point = np.array([1,1])
		self.ref_elem = random.choice(self.env.elements)
		self.goto(self.ref_elem)

	def goto(self,to_elem):
		self.ref_elem = to_elem
		self.ref_point = self.env.get_baricenter(to_elem)

	def is_done(self):
		return self.env.is_element_reached(self.state,self.ref_elem)

	def cont_sim(self,dt):
		d = np.linalg.norm(self.ref_point-self.state)
		if d<1e-6:
			return self.state*0
		n = (self.ref_point-self.state)/d
		self.u = sat(d,1)*self.speed*n

		ContinuousSimObj.cont_sim(self,dt)

	def get_FTS(self):
		nodes = [(e,{'label':set([r]),'weight':1.0}) for e,r in zip(self.env.elements,self.env.regions)]
		L = self.env.elements
		E = list(itertools.permutations(L,2)) + [(l,l) for l in L]
		edges = [(u,v,{'weight':1.0}) for u,v in E]

		symbols = set(self.env.regions)
		print self.ref_elem
		fts = FTS(symbols,set([self.ref_elem]))

		fts.add_nodes_from(nodes)
		fts.add_edges_from(edges)
		return fts

# Subject to synchronization
class DiscreteSimObj:
	def __init__(self):
		self.notice = lambda : None
		self.need_notice = False

	@abstractmethod
	def transition_available(self):
		pass

	def manage_notification(self,sim):
		if self.need_notice:
			self.notice()
			self.need_notice = False

class DrawableObject:
	def __init__(self,s='ro',n="unnamed"):
		self.style = s
		self.name = n
		self.draw = True

class SyncPool:
	def __init__(self,obj_pool,prefix,suffix):
		self.obj_pool = set(obj_pool)
		self.prefix = prefix
		self.suffix = suffix
		self.need_sync = lambda i: self.prefix[i] if i<len(self.prefix) else self.suffix[(i-len(self.prefix))%len(self.suffix)]
		self.get_sync_index = lambda i: i if i<len(self.prefix) else (i-len(self.prefix))%len(self.suffix)

		self.pool = {i:set([]) for i in range(len(self.prefix)+len(self.suffix))}

	def ready(self,obj,id_sync):
		i = self.get_sync_index(id_sync)
		self.pool[i].add(obj)

	def already_noticed(self,obj,id_sync):
		i = self.get_sync_index(id_sync)
		return obj in self.pool[i]

	def sync_available(self,obj,id_sync):
		return self.need_sync(id_sync)

	def is_sync_done(self,id_sync):
		i = self.get_sync_index(id_sync)
		return self.pool[i]==self.obj_pool

	def get_id_sync_done(self):
		L = [i for i,s in self.pool.items() if s==self.obj_pool]
		if L:
			return L[0]
		else:
			return None
	def reset_sync(self,i):
		self.pool[i] = set([])

class InteractiveAgent:
	def __init__(self):
		self.sensor_list = []
		self.meas_labels = set([])

	def add_sensor(self,sensor):
		self.sensor_list.append(sensor)

	def update_measure(self):
		self.meas_labels.clear()
		for s in self.sensor_list:
			self.meas_labels.add(s.measure())