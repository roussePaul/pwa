import networkx as nx



print "-"
G = nx.DiGraph()
G.add_edges_from([(0,1),(1,0),(0,2),(2,1),(2,2)])
for cycle in nx.simple_cycles(G):
	print cycle


print "-"
G = nx.DiGraph()
G.add_edges_from([(0,1),(1,0),(0,2),(2,1),(1,2),(2,2)])
for cycle in nx.simple_cycles(G):
	print cycle

print "-"
G = nx.DiGraph()
G.add_edges_from([(0,0)])
for cycle in nx.simple_cycles(G):
	print cycle


print "Subgraph"
H = G.subgraph([0,1])
print H.nodes()
print H.edges()