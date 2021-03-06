import numpy as np
import random
import itertools
from automata import *
import simulator
import controller
import time

def sat(x,m):
	return x if abs(x)<m else x/abs(x)*m

class Quad(simulator.ContinuousSimObj,simulator.DrawableObject,simulator.InteractiveAgent,object):
#class Quad(controller.ControlledSimObj,simulator.DrawableObject,simulator.InteractiveAgent,object):
	def __init__(self,name,init_region,env):
		#controller.ControlledSimObj.__init__(self)
		simulator.ContinuousSimObj.__init__(self)
		simulator.DrawableObject.__init__(self)
		simulator.InteractiveAgent.__init__(self)

		self.name = name
		self.init_region = init_region
		self.env = env
		elem = self.env.get_elem_in_region(self.init_region)
		self.u = np.array([0,0])
		#self.goto(elem)
		self.state = self.env.get_baricenter(elem)

		self.planner = None

		self.meas = set([])
		self.sensors = []

		self.replan = False

		self.current_region = None

		self.trace = []
		self.transition_type = []
		self.ref_trace = ([],[])

		self.ba = None
		self.measurement_model = None
		self.ts1 = None
		self.prod = None
		self.ltl = None

		self.dst_state = []

	def update_pos(self,sim):
		new_region = self.env.get_region(self.state)
		if new_region!=self.current_region:
			self.current_region = new_region
			self.update_measure(sim)
			self.update_trace()

	def update_measure(self,sim):
		self.meas.clear()
		for m in self.sensors:
			self.meas.add(m.measure(sim))

	def update_trace(self):
		self.trace.append(self.get_labels())
		self.transition_type.append("quad")
		#print "UPDATE TRACE",self.trace

	def get_labels(self):
		return set([self.current_region] + list(self.meas))

	def notice_measurement_transition(self):
		self.trace.append(self.get_labels())
		self.transition_type.append("meas")
		print "NOTICE TRACE",self.trace

	def transition_available(self,sim):
		return self.is_done()

	def action(self,i):
		return self.planner[0][i] if i<len(self.planner[0]) else self.planner[1][(i-len(self.planner[0]))%len(self.planner[1])]

	def di_sim(self,sim):
		trace = self.trace + [self.get_labels()]
		if self.replan:
			self.replanner(trace)
		next_action = self.action(self.planner_state)
		if self.current_action!=next_action:
			self.current_action = next_action
			print self.name," planner[",self.planner_state,"] -> ",self.current_action
			self.planner_state += 1
			self.goto(self.current_action)


	def check_traces(self,trace):
		for i,t in enumerate(trace):
			ref_t = self.ref_trace[0][i] if i<len(self.ref_trace[0]) else self.ref_trace[1][(i-len(self.ref_trace[0]))%len(self.ref_trace[1])]
			if ref_t!=t:
				return False
		return True

	def replanner(self,trace):
		if self.trace:
			self.dst_state  = self.prod.depht_run(self.dst_state,self.trace,self.transition_type)
		if self.check_traces(trace)==False:
			#print self.dst_state
			self.prod.graph['initial'] = set([self.dst_state[-1][0]])

			self.ba = self.ltl.fts_product(self.prod)
			pref,pref_cost,suf,suf_cost= self.ba.dijkstra_plan_networkX(10)
			pr1 = [p[0][0] for p in pref]
			su1 = [s[0][0] for s in suf]
			self.planner = (pr1,su1)
			self.planner_state = 0

			print "New plan ",pref,suf
			
class MonotoneQuad(simulator.ContinuousSimObj,simulator.DrawableObject,simulator.InteractiveAgent,object):
#class Quad(controller.ControlledSimObj,simulator.DrawableObject,simulator.InteractiveAgent,object):
	def __init__(self,name,init_position):
		#controller.ControlledSimObj.__init__(self)
		simulator.ContinuousSimObj.__init__(self)
		simulator.DrawableObject.__init__(self)
		simulator.InteractiveAgent.__init__(self)

		self.name = name

		self.u = np.array([0,0])
		self.state = init_position

		self.planner = None

		self.meas = set([])
		self.sensors = []

		self.replan = False

		self.current_region = None

		self.trace = []
		self.transition_type = []
		self.ref_trace = ([],[])

		self.ba = None
		self.measurement_model = None
		self.ts1 = None
		self.prod = None
		self.ltl = None

		self.dst_state = []

	def update_pos(self,sim):
		pass

	def update_measure(self,sim):
		self.meas.clear()
		for m in self.sensors:
			self.meas.add(m.measure(sim))

	def update_trace(self):
		self.trace.append(self.get_labels())
		self.transition_type.append("quad")
		#print "UPDATE TRACE",self.trace

	def get_labels(self):
		return set([])

	def notice_measurement_transition(self):
		self.trace.append(self.get_labels())
		self.transition_type.append("meas")
		print "NOTICE TRACE",self.trace

	def transition_available(self,sim):
		return self.is_done()

	def action(self,i):
		return self.planner[0][i] if i<len(self.planner[0]) else self.planner[1][(i-len(self.planner[0]))%len(self.planner[1])]

	def check_traces(self,trace):
		for i,t in enumerate(trace):
			ref_t = self.ref_trace[0][i] if i<len(self.ref_trace[0]) else self.ref_trace[1][(i-len(self.ref_trace[0]))%len(self.ref_trace[1])]
			if ref_t!=t:
				return False
		return True


class Measure:
	def __init__(self,callback):
		self.callback = callback

	def measure(self,sim):
		return self.callback(sim)

class MeasureProcess(simulator.DiscreteSimObj,simulator.DrawableObject,object):
	def __init__(self,time_constant,fts,env):
		simulator.DiscreteSimObj.__init__(self)
		simulator.DrawableObject.__init__(self)
		self.time = 0.0
		self.last_transition = 0.0
		self.fts = fts
		self.time_constant = time_constant
		self.fts_state = random.choice(list(self.fts.graph['initial']))
		self.env = env
		self.update_continous_state()


	def transition_available(self,sim):
		return sim.time-self.last_transition>self.time_constant

	def di_sim(self,sim):
		print "Measurement Process transition"
		self.last_transition = sim.time

		l = self.fts.edges(self.fts_state)
		if len(l)>0:
			self.fts_state = random.choice(l)[1]
			self.update_continous_state()
		self.need_notice = True

	def update_continous_state(self):
		self.state = self.env.get_baricenter(self.fts_state)
		self.current_region = self.env.get_region_from_elem(self.fts_state)

	def update_measure(self,sim):
		pass

class CentralizedPlanner(simulator.DiscreteSimObj,object):
	def __init__(self,obj_list,planner):
		simulator.DiscreteSimObj.__init__(self)
		self.obj_list = obj_list
		self.planner = planner
		self.planner_state = 0
		self.goto_next_state()

	def transition_available(self,sim):
		return all([obj.is_done() for obj in self.obj_list])

	def goto_next_state(self):
		for obj in self.obj_list:
			obj.goto(self.action(self.planner_state)[obj.name])
		self.planner_state+=1
	def di_sim(self,sim):
		self.goto_next_state()

	def action(self,i):
		return self.planner[0][i] if i<len(self.planner[0]) else self.planner[1][(i-len(self.planner[0]))%len(self.planner[1])]

class MonotonePlanner(simulator.DiscreteSimObj,object):
	def __init__(self,system,monotone_system,fts,verbose=0):
		simulator.DiscreteSimObj.__init__(self)
		self.fts = fts
		self.monotone_system = monotone_system
		self.period = 0.1
		self.system = system
		self.last_time = 0.0
		self.fts_state = self.init_state()

		self.control = nx.get_node_attributes(self.fts,"apply_control")

		self.verbose = verbose
		self.name = "planner"

	def init_state(self):
		cell = self.monotone_system.get_cell(self.system.get_state())
		for n in self.fts.graph['initial']:
			if cell==n[0]:
				return n

	def get_next_state(self):
		cell = self.monotone_system.get_cell(self.system.get_state())
		for n in self.fts.successors(self.fts_state):
			if cell==n[0]:
				return n
		print cell,self.fts.successors(self.fts_state)

	def transition_available(self,sim):
		return sim.time-self.last_time>=self.period

	def di_sim(self,sim):
		self.last_time = sim.time

		self.fts_state = self.get_next_state()
		if self.fts_state:
			self.system.set_control( self.control[self.fts_state] )
			if self.verbose:
				print self.system.u, self.fts_state
				print sim.time
		else:
			print "No fts state!!"


	def get_buchi_states(self):
		return set([n[1] for n in self.fts.nodes()])

	def plot_controls(self,plt,env,bs,scale=0.1,color='k',offset=np.array([0,0])):
		nodes = [n[0] for n in self.fts.nodes() if n[1]==bs]

		ax = plt.axes()
		for fs in nodes:
			u = self.control[(fs,bs)]
			start = env.get_baricenter(fs) + offset
			if np.all(u==0):
				Circle((start[0],start[1]), radius=scale, color='g', fill=True)
			else:
				x = scale * u/np.linalg.norm(u)
				ax.arrow(start[0], start[1], x[0],x[1], head_width=0.01, head_length=0.02, fc=color, ec=color)

	def show_controls(self,plt,env):
		env.plot(plt)
		color = ['b','r','k','c']
		for i,bs in enumerate(self.get_buchi_states()):
			print bs,"\t",color[i]
			self.plot_controls(plt,env,bs,scale=0.1,color=color[i])
		plt.show()



class ProductSystem:
	def __init__(self,quads):
		self.quads = quads
		self.n = len(quads)
		self.u = None

	def get_state(self):
		return np.concatenate(tuple([q.get_state() for q in self.quads]))

	def set_control(self,u):
		self.u = u
		controls = list(np.reshape(u,(self.n,2)))
		for q,c in zip(self.quads,controls):
			q.set_control(c)
