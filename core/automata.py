from graphviz import Source
from networkx.classes.digraph import DiGraph
from compiler.ast import flatten
import copy
import time

import numpy as np
import random
import itertools
import sys

import os
my_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(my_dir+'/../LTL/')
from buchi import buchi_from_ltl,mission_to_buchi,check_label_for_buchi_edge

import networkx as nx
from networkx.drawing.nx_agraph import to_agraph

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

class Automaton(DiGraph):
	def __init__(self,*args,**kargs):
		DiGraph.__init__(self,*args,**kargs)

	def show(self,name):
		A=to_agraph(self)
		if 'accept' in self.graph:
			for n in self.graph['accept']:
				node = A.get_node(n)
				node.attr['peripheries'] = 2
				
		for n in self.graph['initial']:
			node = A.get_node(n)
			node.attr['penwidth'] = 2
		for n,d in self.nodes(data=True):
			node = A.get_node(n)
			node.attr['label'] = str(n) + "\n" + str(d['label'])
		src = Source(A)
		src.format = 'png'
		src.render("test-output/"+name,view=True)

	def minimize(self):
		for n in self.nodes():
			path_exist = False
			for m in self.graph['initial']:
				if nx.has_path(self,m,n):
					path_exist = True
					break
			if path_exist==False:
				self.remove_node(n)

	def run_trace(self,state_trace):
		labels = nx.get_node_attributes(self,'label')
		return [labels[n] for n in state_trace]


class FTS(Automaton):
	def __init__(self, symbols=set([]),initial=set([])):
		Automaton.__init__(self, symbols=symbols, type="fts", initial=initial)
		self.size = 1 # size of the states of the FTS (used for the product operation)

	def fts_product(self,fts2):
		assert type(fts2) == FTS
		result =  FTS() 
		
		nodes_prod = lambda u,v: tuple((list(u) if self.size>1 else [u]) + (list(v) if fts2.size>1 else [v]))
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

		result.size = self.size + fts2.size

		return copy.deepcopy(result)

	def fts_async_product(self,fts2):
		assert type(fts2) == FTS
		result =  FTS(set([]),set([])) 
		
		nodes_prod = lambda u,v: tuple((list(u) if self.size>1 else [u]) + (list(v) if fts2.size>1 else [v]))
		labels_prod = lambda u,v: flatten((u,v))

		labels = nx.get_node_attributes(self,'label')
		labels_fts2 = nx.get_node_attributes(fts2,'label')

		for u in self.nodes():
			for v in fts2.nodes():
				result.add_node(nodes_prod(u,v),label=labels_prod(labels[u],labels_fts2[v]),weight=1.0)
		for (u,v) in self.edges():
			for (x,y) in fts2.edges():
				result.add_edge(nodes_prod(u,x),nodes_prod(u,y),weight=1.0)
				result.add_edge(nodes_prod(u,x),nodes_prod(v,x),weight=1.0)
				result.add_edge(nodes_prod(u,x),nodes_prod(v,y),weight=1.0)

		for u in self.graph['initial']:
			for v in fts2.graph['initial']:
				result.graph['initial'].add(nodes_prod(u,v))
		
		for u in self.graph['symbols']:
			for v in fts2.graph['symbols']:
				result.graph['symbols'].add(nodes_prod(u,v))

		result.size = self.size + fts2.size

		return copy.deepcopy(result)


	def fts_compose(self,fts2):
		result =  FTS(set([]),set([])) 
		labels = nx.get_node_attributes(self,'label')
		labels_fts2 = nx.get_node_attributes(fts2,'label')
		nodes_prod = lambda u,v: tuple((list(u) if self.size>1 else [u]) + (list(v) if fts2.size>1 else [v]))
		labels_prod = lambda u,v: u.union(v)

		for u in self.nodes():
			for v in fts2.nodes():
				result.add_node(nodes_prod(u,v),label=labels_prod(labels[u],labels_fts2[v]),weight=1.0)

		for (u,v) in self.edges():
			for (x,y) in fts2.edges():
				result.add_edge(nodes_prod(u,x),nodes_prod(u,y),weight=1.0,type=fts2.graph['type'])
				result.add_edge(nodes_prod(u,x),nodes_prod(v,x),weight=1.0,type=self.graph['type'])

		for u in self.graph['initial']:
			for v in fts2.graph['initial']:
				result.graph['initial'].add(nodes_prod(u,v))
		
		result.graph['symbols'] = self.graph['symbols'].union(fts2.graph['symbols'])
		return copy.deepcopy(result)

	def depht_run(self,depht_search_state,trace,transition_type):
		#print "******* START ********"
		labels = nx.get_node_attributes(self,'label')
		edge_type = nx.get_edge_attributes(self,'type')

		if depht_search_state==[]:
			depht_search_state = [(v,None,0) for v in self.graph['initial']]
		idt=0
		#print "TRACE ", trace
		#print "TYPE  ", transition_type
		if len(trace)<=1:
			return depht_search_state
		while len(depht_search_state)>0:
			node,edge,idt = depht_search_state.pop()
			if edge:
				#print " "*idt,node,labels[node],len(depht_search_state),edge_type[edge],transition_type[idt] 
				pass
			else:
				#print " "*idt,node,labels[node],len(depht_search_state),transition_type[idt] 
				pass
			if edge==None:
				for v in self.neighbors(node):
					depht_search_state.append( (v,(node,v),idt+1) )
			elif edge_type[edge]==transition_type[idt] and trace[idt]==labels[node]:
				#print "-->"
				if idt+1==len(trace):
					depht_search_state.append( (node,edge,idt) )
					return depht_search_state

				for v in self.neighbors(node):
					depht_search_state.append( (v,(node,v),idt+1) )

		return depht_search_state

class BA(Automaton):
	def __init__(self,formula="",alpha=100,_type="hard_buchi"):
		if formula == "":
			Automaton.__init__(self,type=_type, initial=set([]), accept=set([]), symbols=set([]))
		else:
			G = buchi_from_ltl(formula,_type)
			Automaton.__init__(self,G)
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

	def trace_run(self,prefix,suffix):
		pref_n = len(prefix)
		suff_n = len(suffix)
		state_trace = lambda ids: tuple(prefix[ids] if ids<pref_n else suffix[(ids-pref_n)%suff_n])
		node_to_visit = [(n,0) for n in self.graph['initial']]
		current_trace = [None]*(3*suff_n+pref_n+1)

		while len(node_to_visit)>0:
			n,ids = node_to_visit.pop()
			s = state_trace(ids)
		
			l1 = list(list(list(n)[0])[0])[0]
			l2 = list(list(list(n)[0])[1])[0]
			l3 = list(n)[1]
			# if s==n[0]:
			# 	if l1==4 and l2==0:
			# 		print n[0],s
			# 	print " "*ids+str(((l1,l2,l3),ids,ids>=pref_n,\
			# 		[list(c)[1] for c in current_trace[pref_n:ids] if c]))
		
			if n in self.graph['accept'] and ids>=pref_n \
				and n in current_trace[pref_n:ids]:
				return True

			if ids<len(current_trace):
				current_trace[ids] = n
				if s==n[0]:
					for v in self.neighbors(n):
						node_to_visit.append((v,ids+1))
		return False

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

		# minimal circles
		for prod_target in self.graph['accept']:
			cycle = {}
			loop_pre, loop_dist = nx.dijkstra_predecessor_and_distance(self, prod_target)
			#print "\nTEST",prod_target
			for target_pred in self.predecessors_iter(prod_target):
				if target_pred in loop_dist:
					#print target_pred, prod_target
					cycle[target_pred] = self.edge[target_pred][prod_target]["weight"] + loop_dist[target_pred]
			if cycle:
				opti_pred = min(cycle, key = cycle.get)
				suffix = compute_path_from_pre(loop_pre, opti_pred)
				#print opti_pred,prod_target,suffix
				loop[prod_target] = (cycle[opti_pred], suffix)

		# print "loops"
		# for p in loop.keys():
		# 	print p,"\t>>",loop[p]
		# shortest line
		for prod_init in self.graph['initial']:
			line = {}
			line_pre, line_dist = nx.dijkstra_predecessor_and_distance(self, prod_init)

			# print
			# print prod_init
			# print "line_dist"
			# for l,m in line_dist.items():
			# 	print l,m

			# print "line_pre"
			# for l,m in line_pre.items():
			# 	print l,m
			# print "loop"
			# for n in loop.iterkeys():
			# 	print n
			# print "\n<<<<<<<<<<<<"
			for target in loop.iterkeys():
				if target in line_dist.iterkeys():
					line[target] = line_dist[target]+beta*loop[target][0]
					# print prod_init,target,line[target]
			# print "<<<<<<<<<<<<\n"
			if line:
				opti_targ = min(line, key = line.get)
				prefix = compute_path_from_pre(line_pre, opti_targ)
				precost = line_dist[opti_targ]
				runs[(prod_init, opti_targ)] = (prefix, precost, loop[opti_targ][1], loop[opti_targ][0])
				# print prod_init,">>",opti_targ, ">>",loop[opti_targ][1]

		# print "Runs\n...................................."
		# for r,p in runs.items():
		# 	print
		# 	print r
		# 	print p
		# print "...................................."
		# best combination
		if runs:
			prefix, precost, suffix, sufcost = min(runs.values(), key = lambda p: p[1] + beta*p[3])
			#print '\n==================\n'
			return prefix, precost, suffix, sufcost
			#print '\n==================\n'
		print 'no accepting run found in optimal planning!'

	def update_unobsarvable_model(self,measurement_model,previous_model,trace):
		print "********Start********"
		print trace
		# (node of the buchi automata, index of the corresponding trace element, number of transitions of the measurement model)
		measurement_init_non_deterministic = 1
		node_to_visit = [(n,0,measurement_init_non_deterministic,False) for n in self.graph['initial']]

		labels = nx.get_node_attributes(previous_model,'label')
		is_measurement_model_transition = lambda u,v: (u[0][1],v[0][1]) in measurement_model.edges()
		measurements_labels = lambda u,v: set(['found','notfound']).intersection(u).intersection(v)
		state_labels = lambda u: u.difference(set(['notfound','found']))

		chemin = [None]*len(trace)
		transitions_measurement_model = [None]*len(trace)
		proj_measurement = lambda u: u[0][1]



		changed = False
		while len(node_to_visit)>0:

			print "Node to visit ", node_to_visit
			# depht search for all possible paths that respect the trace
			node,id_trace,measure_transition_nbr,is_meas_transition = node_to_visit.pop()

			if id_trace<len(trace):

				print "|--"*id_trace,node,trace[id_trace]

				trans_meas = "meas" in trace[id_trace+1] if id_trace+1<len(trace) else False
				if "meas" in trace[id_trace]:
					trace[id_trace].remove("meas")

				chemin[id_trace] = node
				if is_meas_transition and id_trace>0:
					transitions_measurement_model[id_trace] = (chemin[id_trace-1],chemin[id_trace])

				
				measure_prediction_fault = (state_labels(trace[id_trace])==state_labels(labels[node[0]]) and len(measurements_labels(trace[id_trace],labels[node[0]]))==0)

				if trace[id_trace]==labels[node[0]]:
					for v in self.neighbors(node):
						n = measure_transition_nbr
						if trans_meas:
							n += 1
							if is_measurement_model_transition(node,v):
								print "Add ", v
								node_to_visit.append( (v,id_trace+1,n,trans_meas) )
						else:
							print "Add ", v
							node_to_visit.append( (v,id_trace+1,n,trans_meas))

				elif measure_prediction_fault:
					print state_labels(trace[id_trace]), state_labels(labels[node[0]])
					print "Traces are wrong at ",id_trace, trace[id_trace],labels[node[0]]
					# Modify model
					if (measure_transition_nbr==1):
						if chemin[0][0][1] in measurement_model.graph['initial']:
							print "//\\\\ Remove ",chemin[0][0][1]
							measurement_model.graph['initial'].remove(chemin[0][0][1])
						changed = True
					else:
						tr = [ e for e in transitions_measurement_model[:id_trace] if e]
						for (x,y) in tr:
							u = proj_measurement(x)
							v = proj_measurement(y)
							print u,v
							print "//\\\\ edges",measurement_model.edges()
							if measure_transition_nbr==1:
								measurement_model.remove_edge((u,v))
								changed = True
							elif measure_transition_nbr>1:
								measurement_model[u][v]['weight'] += 1/(measure_transition_nbr-1)
								changed = True

		return changed
