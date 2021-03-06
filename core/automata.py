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

import collections
from sortedcontainers import SortedList

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

	def show(self,name,colors={},edges_color={}):
		A=to_agraph(self)
		try:
			for n,d in self.nodes(data=True):
				node = A.get_node(n)
				if 'label' in d:
					node.attr['label'] = str(n) + "\n" + str(d['label'])
			for n,c in colors.items():
				node = A.get_node(n)
				node.attr['fillcolor'] = c
				node.attr['style'] = 'filled'

			for n,c in edges_color.items():
				node = A.get_edge(*n)
				node.attr['color'] = c
				node.attr['penwidth'] = 2

			if 'accept' in self.graph:
				for n in self.graph['accept']:
					node = A.get_node(n)
					node.attr['peripheries'] = 2
					
			for n in self.graph['initial']:
				node = A.get_node(n)
				node.attr['penwidth'] = 2

		except Exception,e:
			print "Error on printing graph: ",e
		src = Source(A)
		src.format = 'png'
		src.render("test-output/"+name,view=True)

	def compact_show(self,name,colors={},edges_color={}):
		A=to_agraph(self)
		try:
			if 'accept' in self.graph:
				for n in self.graph['accept']:
					node = A.get_node(n)
					node.attr['peripheries'] = 2
					
			for n in self.graph['initial']:
				node = A.get_node(n)
				node.attr['penwidth'] = 2
			for n,d in self.nodes(data=True):
				node = A.get_node(n)
				if 'label' in d:
					node.attr['label'] = str(n) + "\n" + str(d['label'])
			for n,c in colors.items():
				node = A.get_node(n)
				node.attr['color'] = c
				node.attr['style'] = 'filled'
				print n,c
			for n,c in edges_color.items():
				node = A.get_edge(*n)
				node.attr['color'] = c
				node.attr['penwidth'] = 2
				print n,c

		except Exception,e:
			print "Error on printing graph: ",e
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
				self.remove_node_init_accept(n)

	def run_trace(self,state_trace):
		labels = nx.get_node_attributes(self,'label')
		return [labels[n] for n in state_trace]

	def remove_node_init_accept(self,n):
		self.remove_node(n)
		if n in self.graph['initial']:
			self.graph['initial'].remove(n)
		if n in self.graph['accept']:
			self.graph['accept'].remove(n)

	def remove_labeled_edge(self,n,label):
		for v in self.neighbors(n):
			if label in self[n][v]['label']:
				self[n][v]['label'].remove(label)
				if len(self[n][v]['label'])==0:
					self.remove_edge(n,v)

	def remove_labeled_edge_except(self,n,label):
		label_to_remove = self.get_successor_labels(n).difference(set([label]))
		for l in label_to_remove:
			self.remove_labeled_edge(n,l)

	def get_successor_labels(self,node):
		labels = set([])
		for v in self.neighbors(node):
			labels = labels.union(self[node][v]['label'])
		return labels

	def get_labelled_successors(self,node):
		d = {l:[] for l in self.get_successor_labels(node)}
		for v in self.neighbors(node):
			for l in d.keys():
				if l in self[node][v]['label']:
					d[l].append(v)
		return d

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
							if ((t_ts_node in fts.graph['initial']) and (t_buchi_node in self.graph['initial'])):
								result.graph['initial'].add(t_prod_node)
							if (t_buchi_node in self.graph['accept']):
								result.graph['accept'].add(t_prod_node)
							# Meng one: label = fts.node[f_ts_node]['label']
							label = fts.node[t_ts_node]['label']
							cost = fts[f_ts_node][t_ts_node]['weight']
							truth, dist = check_label_for_buchi_edge(self, label, f_buchi_node, t_buchi_node)
							total_weight = cost + self.alpha*dist

							if truth:
								result.add_edge(f_prod_node, t_prod_node, weight=total_weight)
		return result

	def undeterministic_fts_product(self,fts,verbose=0):
		result = BA()

		node_edge_labels = {u:list(set(itertools.chain(*[l['label'] for l in fts[u].values()]))) for u in fts.nodes()}
		edge_labels = {u:[(l,[v for v,k in fts[u].iteritems() if l in k['label']]) for l in node_edge_labels[u]] for u in fts.nodes()}
		removed_labels = {}


		N_nodes = len(fts.nodes())
		i_node = 0
		for f_ts_node in fts.nodes_iter():
			i_node +=1
			if verbose:
				sys.stdout.write(str(i_node)+"/" + str(N_nodes)+'\r')
				sys.stdout.flush()
			for f_buchi_node in self.nodes_iter():
				f_prod_node = self.composition(result,fts,f_ts_node, f_buchi_node)
				if f_prod_node not in removed_labels:
					removed_labels[f_prod_node] = {}
				for l,successor in edge_labels[f_ts_node]:
					for t_buchi_node in self.successors_iter(f_buchi_node):
						all_succ_ok = True
						for t_ts_node in successor:
							t_prod_node = self.composition(result,fts,t_ts_node, t_buchi_node)
							label = fts.node[t_ts_node]['label']
							cost = fts[f_ts_node][t_ts_node]['weight']
							truth, dist = check_label_for_buchi_edge(self, label, f_buchi_node, t_buchi_node)
							if truth==False:
								all_succ_ok = False
								break
						if all_succ_ok:
							for t_ts_node in successor:
								t_prod_node = self.composition(result,fts,t_ts_node, t_buchi_node)
								label = fts.node[t_ts_node]['label']
								cost = fts[f_ts_node][t_ts_node]['weight']
								truth, dist = check_label_for_buchi_edge(self, label, f_buchi_node, t_buchi_node)
								total_weight = cost + self.alpha*dist
								if truth:
									new_label = copy.copy(fts[f_ts_node][t_ts_node]['label'])
									new_control = copy.copy(fts[f_ts_node][t_ts_node]['control'])
									if (f_prod_node,t_prod_node) in result.edges():
										new_label = new_label.union(result[f_prod_node][t_prod_node]['label'])
										new_control.update(result[f_prod_node][t_prod_node]['control'])
									result.add_edge(f_prod_node, t_prod_node, weight=total_weight,label=new_label,control=new_control)

									if ((t_ts_node in fts.graph['initial']) and (t_buchi_node in self.graph['initial'])):
										result.graph['initial'].add(t_prod_node)
									if (t_buchi_node in self.graph['accept']):
										result.graph['accept'].add(t_prod_node)
						else:
							for t_ts_node in successor:
								t_prod_node = self.composition(result,fts,t_ts_node, t_buchi_node)
								if t_prod_node not in removed_labels[f_prod_node]:
									removed_labels[f_prod_node][t_prod_node] = []
								removed_labels[f_prod_node][t_prod_node].append(l)

		for u,v in result.edges():
			if u in removed_labels and v in removed_labels[u]:
				result[u][v]['label'] = result[u][v]['label'].difference(set(removed_labels[u][v]))

		return result


	def get_states_connected_to_accept(self):
		S = set([])
		for n in self.graph['accept']:
			pre = list(itertools.chain(*nx.predecessor(self,n).values()))
			S = S.union(set(pre))
		return S

	def fixed_point(self,S):
		node_edge_labels = {u:list(set(itertools.chain(*[l['label'] for l in self[u].values()]))) for u in self.nodes()}
		edge_labels = {u:[(l,[v for v,k in self[u].iteritems() if l in k['label']]) for l in node_edge_labels[u]] for u in self.nodes()}

		accepted_states = set([])
		for n in S:
			for l,successor in edge_labels[n]:
				if S.issuperset(set(successor)):
					accepted_states.add(n)
		
		return S.intersection(accepted_states)

	def get_safe_states(self):
		S = self.get_states_connected_to_accept()
		old_S = set([])
		while old_S!=S:
			old_S = S
			print len(old_S),len(S)
			S = self.fixed_point(S)

		node_edge_labels = {u:list(set(itertools.chain(*[l['label'] for l in self[u].values()]))) for u in self.nodes()}
		edge_labels = {u:[(l,[v for v,k in self[u].iteritems() if l in k['label']]) for l in node_edge_labels[u]] for u in self.nodes()}
		suppressed_nodes = set(self.nodes()).difference(S)
		suppressed_edges = set([(u[0],l) for u,k in edge_labels.items() for l,successor in k if len(set(successor).intersection(suppressed_nodes))>0])

		return S,suppressed_nodes,suppressed_edges

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

	def get_fts_plan(self):
		# BFS algorithm from accepted states to write distance from all nodes
		fts = FTS()

		fts.add_edges_from(self.edges(data=True))


		cycle_edges = []
		for u in self.graph['accept']:
			cycle = []
			# print "------------",u,"-----------------"
			paths = nx.shortest_path(fts,target=u).values()
			for p in paths:
				for v in self.successors(u):
					if v in p:
						cycle.append((p,(u,v)))
						# print u,v,p
			c = min(cycle, key=lambda l: len(l[0]))
			# print 
			# print min([len(l[0]) for l in cycle])
			# print [len(l[0]) for l in cycle]
			# print "Smallest one",c
			cycle_edges.append(c[1])

		fake_state = [(u,'final') for u in self.graph['accept']]
		fts.add_edges_from(fake_state)
		paths = nx.shortest_path(fts,target='final').values()
		paths = [p[:-1] for p in paths]

		usefull_edges = list(itertools.chain(*[[(u,v) for u,v in zip(p[:-1],p[1:])] for p in paths] )) + cycle_edges
		unusefull_edges = set(fts.edges()).difference(set(usefull_edges))

		fts.remove_edges_from(list(unusefull_edges))
		fts.remove_node('final')


		for u,v in fts.edges():
				l,c = fts[u][v]['control'].items()[0]
				fts[u][v]['control'] = c
				fts[u][v]['label'] = set([l])


		model_plan = copy.copy(self)
		for n,d in model_plan.nodes(data=True):
			d['apply_control'] = fts.out_edges(n,data=True)[0][2]['control']
		return fts, model_plan

	def is_cycle_fair(self,system,cycle):
		ls =  [system.get_linear_system(c[0][0]) for c in cycle]
		A = [l[0] for l in ls]
		A_tot = reduce(np.dot,A)
		u = [c[1] for c in cycle]
		func = lambda p,a: np.dot(a[0],p)+np.dot(a[1],a[2])
		L = [(a[0],a[1],b) for a,b in zip(A,u)]
		b_tot = reduce(func,L,0.0*L[0][2])
		if np.all(A_tot==np.eye(A_tot.shape[0])) and np.all(b_tot!=0.0):
			return True
		for c in cycle:
			elem = c[0][0]
			if system.is_in_cell(elem,b_tot):
				return False
		return True

	def self_loop_fairness(self,system,node,label):
		u = self[node][node]['control'][label]
		if np.all(u==0):
			return False
		return True

	def create_fair_graph(self,system):
		G = DiGraph()
		G.add_edges_from(self.edges(data=True))
		controls = nx.get_edge_attributes(self,'control')
		unfair_cycles = []
		for cycle in nx.simple_cycles(G):
			edges = [(cycle[i],cycle[(i+1)%len(cycle)]) for i in range(len(cycle))]
			trace = [(c[0],controls[c].values()) for c in edges]
			nbre_controls = [range(len(t[1])) for t in trace]
			control_configuration = itertools.product(*nbre_controls)
			for conf in control_configuration:
				current_trace = [(t[0],t[1][conf[i]]) for i,t in enumerate(trace)]
				if not self.is_cycle_fair(system,current_trace):
					unfair_cycles.append(current_trace)
		print "Unfair cycles ",unfair_cycles

	def plan(self,system):
		# BFS algorithm from accepted states to write distance from all nodes
		fts = FTS()

		fts.add_edges_from(self.edges(data=True))
		fts.graph = copy.copy(self.graph)


		nx.set_node_attributes(fts,'distance',float('Inf'))
		distance = nx.get_node_attributes(fts,'distance')

		node_to_visit = collections.deque()
		for n in self.graph['accept']:
			distance[n] = 0.0
			node_to_visit.append(n)

		while len(node_to_visit)>0:
			n = node_to_visit.popleft()
			print "."+" "*int(distance[n]),n
			for u in [u for u,v in fts.in_edges(n)]:

				if u not in self.graph['accept'] and distance[u]<float('Inf'):
					print "Node already visited",u
					continue
				# if we don't take a self transition on the FTS:
				is_transition_fair = (u[0]!=n[0])

				# we only take the labels that can reach n
				labelled_edges = fts.get_labelled_successors(u)
				labelled_edges = {l:s for l,s in labelled_edges.items() if n in s}

				# -- Cas 1
				# if all successor can go to the next accepted transition
				distance_to_accepted_set = {l:sum([distance[s] for s in succ]) for l,succ in labelled_edges.items()}
				deterministic_transition = [l for l,succ in labelled_edges.items() if distance_to_accepted_set[l]!=float('Inf')]

				# priorite a la tranbsition deterministic
				if len(deterministic_transition)>0:
					chosen_label = min(distance_to_accepted_set,key=distance_to_accepted_set.get)
					fts.remove_labeled_edge_except(u,chosen_label)
					distance[u] = distance[n]+1.0
					node_to_visit.append(u)

					print "+ deterministic",u,"-->",n,chosen_label, deterministic_transition

					continue

				# -- Cas 2
				# self loop, we directely check for fairness of the transition
				# depending on the transition, take the fair transition or the unfair one
				distance_self_loops = {l:sum([distance[s] for s in succ if distance[s]!=float('Inf') ]) for l,succ in labelled_edges.items()}
				self_loops = [l for l,succ in labelled_edges.items() if all([distance[s]!=float('Inf') or s==u for s in succ])]
				# compute all the self loops that will go back to initial position
				fair_self_loops = [l for l,succ in labelled_edges.items() if u in succ and self.self_loop_fairness(system,u,l)]
				if len(self_loops)>0:
					if is_transition_fair:
						valid_edges = {l:distance_self_loops[l] for l in distance_self_loops.keys() if l in fair_self_loops}
					else:
						valid_edges = {l:distance_self_loops[l] for l in distance_self_loops.keys() if l not in fair_self_loops}
					chosen_label = min(valid_edges,key=valid_edges.get)
					fts.remove_labeled_edge_except(u,chosen_label)
					
					if not is_transition_fair:
						fts.remove_edge(u,u)
					
					distance[u] = distance[n]+1.0
					node_to_visit.append(u)

					print "+ self loop",u,"-->",n,chosen_label
					#print distance_self_loops,is_transition_fair,fair_self_loops
					continue

				# -- Cas 3
				# either a cycle (not a self loop), either bring deterministicly to the acceptance set


		nx.set_node_attributes(fts,'apply_control',0)
		control = nx.get_node_attributes(fts,'apply_control')
		label = nx.get_edge_attributes(fts,'label')
		for u,v,d in fts.edges(data=True):
			l = list(label[u,v])[0]
			control[u] = d['control'][l]
		nx.set_node_attributes(fts,'apply_control',control)
		return fts


	def dijkstrat_plan_synthesis(self,system):
		# BFS algorithm from accepted states to write distance from all nodes
		fts = FTS()

		fts.add_edges_from(self.edges(data=True))
		fts.graph = copy.copy(self.graph)


		nx.set_node_attributes(fts,'distance',float('Inf'))
		distance = nx.get_node_attributes(fts,'distance')

		node_to_visit = collections.deque()
		for n in self.graph['initial']:
			distance[n] = 0.0
			node_to_visit.append(n)

		while len(node_to_visit)>0:
			n = node_to_visit.popleft()
			print "."+" "*int(distance[n]),n

			labelled_edges = fts.get_labelled_successors(u)
			for label,succ in labelled_edges.items():

				# Do we have a loop ?
				looping_transition = [v for v in succ if distance[v]<float('Inf')]

				# if we don't take a self transition on the FTS:
				fair_transition = [v[0]!=n[0] for v in succ]

				# we only take the labels that can reach n
				labelled_edges = fts.get_labelled_successors(u)
				labelled_edges = {l:s for l,s in labelled_edges.items() if n in s}

				# -- Cas 1
				# if all successor can go to the next accepted transition
				distance_to_accepted_set = {l:sum([distance[s] for s in succ]) for l,succ in labelled_edges.items()}
				deterministic_transition = [l for l,succ in labelled_edges.items() if distance_to_accepted_set[l]!=float('Inf')]

				# priorite a la tranbsition deterministic
				if len(deterministic_transition)>0:
					chosen_label = min(distance_to_accepted_set,key=distance_to_accepted_set.get)
					fts.remove_labeled_edge_except(u,chosen_label)
					distance[u] = distance[n]+1.0
					node_to_visit.append(u)

					print "+ deterministic",u,"-->",n,chosen_label, deterministic_transition

					continue

				# -- Cas 2
				# self loop, we directely check for fairness of the transition
				# depending on the transition, take the fair transition or the unfair one
				distance_self_loops = {l:sum([distance[s] for s in succ if distance[s]!=float('Inf') ]) for l,succ in labelled_edges.items()}
				self_loops = [l for l,succ in labelled_edges.items() if all([distance[s]!=float('Inf') or s==u for s in succ])]
				# compute all the self loops that will go back to initial position
				fair_self_loops = [l for l,succ in labelled_edges.items() if u in succ and self.self_loop_fairness(system,u,l)]
				if len(self_loops)>0:
					if is_transition_fair:
						valid_edges = {l:distance_self_loops[l] for l in distance_self_loops.keys() if l in fair_self_loops}
					else:
						valid_edges = {l:distance_self_loops[l] for l in distance_self_loops.keys() if l not in fair_self_loops}
					chosen_label = min(valid_edges,key=valid_edges.get)
					fts.remove_labeled_edge_except(u,chosen_label)
					
					if not is_transition_fair:
						fts.remove_edge(u,u)
					
					distance[u] = distance[n]+1.0
					node_to_visit.append(u)

					print "+ self loop",u,"-->",n,chosen_label
					#print distance_self_loops,is_transition_fair,fair_self_loops
					continue

				# -- Cas 3
				# either a cycle (not a self loop), either bring deterministicly to the acceptance set


		nx.set_node_attributes(fts,'apply_control',0)
		control = nx.get_node_attributes(fts,'apply_control')
		label = nx.get_edge_attributes(fts,'label')
		for u,v,d in fts.edges(data=True):
			l = list(label[u,v])[0]
			control[u] = d['control'][l]
		nx.set_node_attributes(fts,'apply_control',control)
		return fts

	# remove all the states that have the same observation and same label transition
	# in case of conflict we remove the transition that is not on an accepted states
	def remove_ambiguites(self):
		remove_edges = []
		for u in self.nodes():
			labelled_edges = self.get_labelled_successors(u)
			for l,succ in labelled_edges.items():
				if u in succ and any([u[0]==v[0] and u!=v for v in succ]):
					if u in self.graph['accept']:
						r = v
					else:
						r = u
					remove_edges.append( ((u,r),l) )
		for t,l in remove_edges:
			self[t[0]][t[1]]['label'].remove(l)
			if len(self[t[0]][t[1]]['label'])==0:
				self.remove_edge(*t)

	def generate_all_predecessor_set(self,node_set): 
		return [{u:l}  for n in node_set for u,v,d in self.in_edges(n,data=True) if u not in node_set for l in d['label']]

	def backward_reachability(self,system,inputs,show_dfs=False,verbose=0,max_fixed_point_size=4,environment=None,plt=None):
		accepted_set = self.graph['accept']

		initial_state_found = False
		accepted_set_circled = False

		control_dict = {str(u):u for u in inputs}

		initial_control_set = set([(a,None) for a in accepted_set])
		initial_accepted_set = accepted_set
		initial_fixed_point_iterator = DijkstraFixedPoint(self,self.generate_all_predecessor_set(initial_accepted_set),initial_accepted_set)

		to_visit = collections.deque([(initial_control_set,initial_fixed_point_iterator)])

		iter_loop = 0
		while to_visit and not (initial_state_found and accepted_set_circled):
			e = to_visit[-1]

			accepted_control,fixed_point_iterator = e
			accepted_set = set([x[0] for x in accepted_control])

			if verbose==2:
				print " "*len(to_visit) + "| iter "+str(len(to_visit))


			if verbose==1:
				pass


			found,nodes = fixed_point_iterator.next_fixed_point(max_fixed_point_size)
			
			if not found:
				if len(to_visit)>1:
					to_visit.pop()
				else:
					print "No solution found"
					return

			iter_loop += 1

			if found:
				#print "."*len(accepted_set) + "|"*len(nodes)

				outgoing_edges = [(u,v) for u,l in nodes.items() for v in self.get_labelled_successors(u)[l] if v in accepted_set]
				#need_fairness = not all([u[0]==v[0] for u,v in outgoing_edges])
				need_fairness = not all([any([v[0]==u[0] for v in self.get_labelled_successors(n)[l] if (v in accepted_set) and v!=u]) for n,l in nodes.items() for u in self.get_labelled_successors(n)[l] if u not in accepted_set])
				if verbose==3:
					print "Need Fairness:",need_fairness

					if need_fairness:
						print "!!!!!!!!!!!!!!!!!!", iter_loop
				controls = [control_dict[l] for l in nodes.values()]

				sG = nx.DiGraph()
				# convert = {n:i for i,n in enumerate(nodes.keys())}
				# edges = []
				keep_edges = [(u,v)for u,l in nodes.items() if l!=None for v in self.get_labelled_successors(u)[l]]
				sG = nx.DiGraph(self.subgraph(nodes.keys()))
				sG.remove_edges_from(set(sG.edges()).difference(set(keep_edges)))

				try:
					nx.set_node_attributes(sG,'control',{n:control_dict[l] for n,l in nodes.items() if n in sG.nodes()})
				except KeyError:
					print sG.edges()
					print nodes.keys()
					raise KeyError

				# check if accepted states are accessible from every nodes:
				# print nx.is_strongly_connected(sG),self.subgraph(nodes.keys()).edges()
				if nx.is_strongly_connected(sG)==False:
					continue

				if verbose==2:
					print sG.edges(data=True)
				
				#fair_control = system.is_loop_fair(controls)
				fair_control = system.is_graph_fair(sG)

				if fair_control or need_fairness==False:
					if verbose==2:
						print "need_fairness",need_fairness,"fair_control",fair_control,[str(c) for c in controls]
						print "Add fair loop",nodes
					X = accepted_control.union(set(nodes.items()))
					node_set = [x[0] for x in X]
					Y =  self.generate_all_predecessor_set(node_set)
					it = DijkstraFixedPoint(self,Y,node_set)

					to_visit.append((X,it))

				else:
					if verbose==2:
						print "Unfair loop",controls

				new_set = accepted_set.union(set(nodes.keys()))
				if self.graph['initial'].issubset(new_set):
					initial_state_found = True
					if verbose==2:
						print "Path to initial_set found!!!!"

				cycle_accept_set = [(u,l) for u in self.graph['accept'] for l,succ in self.get_labelled_successors(u).items() if set(succ).issubset(new_set)]

				if environment:
					soluce = accepted_control.union(set(nodes.items()))
					print soluce
					node_controls = {n:(control_dict[l] if l else None) for n,l in soluce}
					environment.show_controls(plt,node_controls,"automaton/iter"+str(iter_loop)+".png")

					print "Accept cycle",len(cycle_accept_set),"Initial set found",self.graph['initial'].issubset(new_set)
					
				if cycle_accept_set and self.graph['initial'].issubset(new_set):
					soluce = accepted_control.union(set(nodes.items()))
					print "Solution found!!!"
					print sG.edges()
					break

				if verbose==0:
					sys.stdout.write(str(len(accepted_set)) +"/" +str(len(nodes)) + "\t" + str(iter_loop)+'\r')
					sys.stdout.flush()


				if show_dfs:
					c = {n:"red" for n in accepted_set}
					c.update({n:"green" for n in nodes})
					ec = {e:"red" for e in [(u,v) for u,l in accepted_control if l!=None for v in self.get_labelled_successors(u)[l]]}
					ec.update( {e:"blue" for e in [(u,v) for u,l in nodes.items() if l!=None for v in self.get_labelled_successors(u)[l]]} )
					self.show("buchi",colors=c,edges_color=ec)
					raw_input()


		print len(to_visit)
		if verbose==3:
			G = copy.deepcopy(self)
			keep_edges = [(u,v)for u,l in nodes.items() if l!=None for v in self.get_labelled_successors(u)[l]]
			keep_nodes = [n for e in keep_edges for n in e]
			print keep_nodes
			G.remove_nodes_from(list(set(G.nodes()).difference(set(keep_nodes))))
			G.remove_edges_from(list(set(G.edges()).difference(set(keep_edges))))
			G.graph['accept'] = G.graph['accept'].intersection(set(keep_nodes))
			G.graph['initial'] = G.graph['initial'].intersection(set(keep_nodes))
			c = {n:"red" for n in accepted_set.intersection(set(keep_nodes))}
			c.update({n:"green" for n in set(nodes.keys()).intersection(set(keep_nodes))})
			ec = {e:"blue" for e in [(u,v) for u,l in nodes.items() if l!=None for v in self.get_labelled_successors(u)[l]] if e in keep_edges}

			G.show("buchi",colors=c,edges_color=ec)


		cycle_accept_set = [(u,l) for u in self.graph['accept'] for l,succ in self.get_labelled_successors(u).items() if set(succ).issubset(new_set)]
		if verbose==2:
			print cycle_accept_set
		solution = {u:l for u,l in soluce}
		for u,l in cycle_accept_set:
			solution[u] = l
		if verbose==2:
			print solution


		plan = copy.deepcopy(self)
		for n,l in solution.items():
			plan.remove_labeled_edge_except(n,l)
		#plan.show("plan_all")
		plan.remove_ambiguites()
		plan.minimize()

		# plan.show("plan2")
		# return 

		if verbose==2:
			print self.graph['initial']

		nx.set_node_attributes(plan,"apply_control",None)
		control = nx.get_node_attributes(plan,"apply_control")
		for n in plan.nodes():
			c = list(plan.get_successor_labels(n))
			if len(c)==0:
				print "Control not found:", n, s
			else:
				s = c[0]
				if len(c)!=1:
					print "Too many controls",n,c
					s = c[0] 
			control[n] = control_dict[s]

		nx.set_node_attributes(plan,"apply_control",control)

		return plan
# parcours de tous les fix point en BFS avec commme depht le nombre de noeuds du fix point
# initial_set = [{n1:l1,n2:l2, ...},...]
class DijkstraFixedPoint:
	def __init__(self, automaton, initial_set, accepted_set):
		self.automaton = automaton
		self.set_to_visit = SortedList(initial_set,key= lambda d: -len(d))
		self.accepted_set = accepted_set

	def iter_fix_point_set(self,max_size=10):
		if len(self.set_to_visit)==0:
			raise StopIteration()

		F = self.set_to_visit.pop()

		nF = {k:[v] for k,v in F.items()}
		new_size_of_fp = len(nF)
		reach_accepted_set = False
		for u,lu in F.items():
			labelled_edges = self.automaton.get_labelled_successors(u)
			succ = labelled_edges[lu]
			for s in succ:
				if s in self.accepted_set:
					reach_accepted_set = True
				if (s not in nF) and (s not in self.accepted_set):
					nF[s] = list(self.automaton.get_successor_labels(s))
					new_size_of_fp = len(nF)
					if new_size_of_fp>max_size:
						return False,F


		newF = self.expand_successor_set(nF)
		if F in newF:
			newF.remove(F)
		self.set_to_visit.update(newF)
		accept_fix_point = (len(newF)==0) and reach_accepted_set
		return accept_fix_point,F


	def expand_successor_set(self,nF):
		sF = []
		# import operator
		# size = reduce(operator.mul, [len(v) for v in nF.values()], 1)
		for conf in itertools.product(*nF.values()):
			sF.append({k:v for k,v in zip(nF.keys(),conf)})
		return sF

	def __iter__(self):
		return self

	def next(self):
		return self.iter_fix_point_set()

	def next_fixed_point(self,max_size):
		fp_found = 0
		try:
			while fp_found==False:
					fp_found,fp = self.iter_fix_point_set(max_size)
					#print "#"*len(fp)
		except StopIteration:
			return False,None
		return fp_found,fp
