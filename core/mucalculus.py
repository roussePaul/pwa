from automata import FTS
import copy
import networkx as nx

class Variable:
	def __init__(self,name):
		self.name = name
		self.T = None

	def __repr__(self):
		return "%s"%(self.name)

	def ffp(self,sys):
		return self.T

class AtomicProposition:
	def __init__(self,name):
		self.name = name

	def __repr__(self):
		return "AP(%s)" % str(self.name)

	def ffp(self,sys):
		s = set([n for n,d in sys.nodes(data=True) if self.name in d['label']])
		return s

class GreatestFixPoint:
	def __init__(self,variable,inner):
		self.variable = variable
		self.inner = inner

	def __repr__(self):
		mu = unichr(0x3bd).encode('utf-8')
		return "%s(%s)" % (mu+str(self.variable)+".",str(self.inner))

	def ffp(self,sys):
		v = self.variable
		v.T = set([])
		T_prec = None
		while v.T!=T_prec:
			T_prec = copy.copy(v.T)
			S = self.inner.ffp(sys)
			v.T = v.T.union(S)

			print "*",v.T,"=",T_prec,"u", S
		return v.T

class LeastFixPoint:
	def __init__(self,variable,inner):
		self.variable = variable
		self.inner = inner

	def __repr__(self):
		nu = unichr(0x3bc).encode('utf-8')
		return "%s(%s)" % (nu+str(self.variable)+".",str(self.inner))

	def ffp(self,sys):
		v = self.variable
		v.T = set(sys.nodes())
		T_prec = None
		while v.T!=T_prec:
			T_prec = copy.copy(v.T)
			S = self.inner.ffp(sys)
			v.T = v.T.intersection(S)
			print "*",v.T,"=",T_prec,"n", S
		return v.T


class Not:
	def __init__(self,inner):
		self.inner = inner

	def __repr__(self):
		return "Not(%s)" % str(self.inner)

	def ffp(self,sys):
		return set(sys.nodes()).difference(self.inner.ffp(sys))

class All:
	def __init__(self,inner):
		self.inner = inner

	def __repr__(self):
		return "All(%s)" % str(self.inner)

	def ffp(self,sys):
		T = self.inner.ffp(sys)
		s = set()
		for u in sys.nodes():
			if all([(v in T) for v in sys.neighbors(u)]):
				s.add(u)
		return s

class Some:
	def __init__(self,inner):
		self.inner = inner

	def __repr__(self):
		return "Some(%s)" % str(self.inner)

	def ffp(self,sys):
		T = self.inner.ffp(sys)
		return set([u for u,v in sys.edges() if v in T])
		
class AllType:
	def __init__(self,_type,inner):
		self.inner = inner
		self.type = _type

	def __repr__(self):
		return "All(%s,%s)" % (str(self.type),str(self.inner))

	def ffp(self,sys):
		T = self.inner.ffp(sys)
		s = set()
		_type = nx.get_edges_attributes(sys,'type')
		for u in sys.nodes():
			if all([(v in T) for v in sys.neighbors(u) if _type[u][v]==self.type]):
				s.add(u)
		return s
		
class SomeAllType:
	def __init__(self,opponant,player,inner):
		self.inner = inner
		self.opponant = opponant
		self.player = player

	def __repr__(self):
		return "SomeAll(%s vs %s,%s)" % (str(self.opponant),str(self.player),str(self.inner))

	def ffp(self,sys):
		T = self.inner.ffp(sys)
		s = set()
		_type = nx.get_edge_attributes(sys,'type')
		for u in sys.nodes():
			a = [v for v in sys.neighbors(u) if _type[(u,v)]==self.opponant]
			m = [any([(w in T) for w in sys.neighbors(v) if _type[(v,w)]==self.player])]
			if all(m):
				s.add(u)
		return s

class And:
	def __init__(self,left,right):
		self.left = left
		self.right = right

	def __repr__(self):
		return "And(%s,%s)" % (str(self.left),str(self.right))

	def ffp(self,sys):
		return self.left.ffp(sys).intersection(self.right.ffp(sys))
		
class Or:
	def __init__(self,left,right):
		self.left = left
		self.right = right

	def __repr__(self):
		return "Or(%s,%s)" % (str(self.left),str(self.right))

	def ffp(self,sys):
		return self.left.ffp(sys).union(self.right.ffp(sys))

P = AtomicProposition("P")
Q = AtomicProposition("Q")
Z = Variable("Z")
Y = Variable("Y")
Y1 = Variable("Y1")
Y2 = Variable("Y2")
#formula = LeastFixPoint(Z,And(P,All(Z)))
formula = GreatestFixPoint(Z,And(P,All(Z)))
print formula

nodes = set([1,2,3,4,5])
edges = [(1,2),(2,3),(3,1),(3,4),(4,5)]
labels = {1:set(["P"]),2:set(["Q"]),3:set([""]),4:set(["Q"]),5:set(["Q"])}

# nodes = set([1,2,3])
# edges = [(1,2),(2,3),(3,1)]
# labels = {1:set(["P"]),2:set([""]),3:set([""]),4:set(["Q"])}

# nodes = set([1,2,3,4])
# edges = [(1,4),(3,1),(1,2),(2,3),(4,3)]
# labels = {1:set([]),2:set([]),3:set([]),4:set(["P"])}

# sys = FTS()
# sys.add_nodes_from([(n,{'label':labels[n]}) for n in nodes])
# sys.add_edges_from(edges)
# sys.show("lkjl")

G = nx.grid_2d_graph(5,5)
labels = {n:set([]) for n in G.nodes()}
labels[(0,0)] = set(["P"])
labels[(4,4)] = set(["Q"])
t = lambda u,v: "a" if abs(u[0]-u[1])<3 else "b"
types = {n:dict() for n in G.nodes()}

edges = [(v,u) for u,v in G.edges()] + G.edges()

for u,v in edges:
	types[u][v] = t(u,v)

sys = FTS()
sys.add_nodes_from([(n,{'label':labels[n]}) for n in G.nodes()])
sys.add_edges_from([(u,v,{'type':types[u][v]}) for u,v in edges])

formula.ffp(sys)

print formula
print Z.T

formula = GreatestFixPoint(Z,P)
print formula
print formula.ffp(sys)
print

formula = LeastFixPoint(Z,P)
print formula
print formula.ffp(sys)
print


formula = LeastFixPoint(Z,Or(P,All(Z)))
print formula 
print formula.ffp(sys)
print

formula = GreatestFixPoint(Z,And(P,Some (Z)))
print formula 
print formula.ffp(sys)
print

formula = LeastFixPoint(Z,Or(P,SomeAllType("a","b",Z)))
print formula 
r = formula.ffp(sys)
print r
print

sys.graph['initial'] = r
sys.show("lkjl")