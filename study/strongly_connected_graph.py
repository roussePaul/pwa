import networkx as nx
import matplotlib.pyplot as plt


import sys
while True:
	print "kjh"
	print input()

G = nx.DiGraph()

G.add_edges_from([(1,2),(2,3),(3,1),(4,2)])
print G.edges()
print nx.is_strongly_connected(G)

print "--"

G = nx.DiGraph()
G.add_edges_from([(1,2),(2,3),(3,1)])
print G.edges()
print nx.is_strongly_connected(G)

edges = [(((21, 22, 27, 28), u'T0_init'), ((21, 22, 27, 28), u'T0_init')), (((21, 22, 27, 28), u'T0_init'), ((14, 15, 20, 21), u'T0_init')), (((21, 22, 27, 28), u'T0_init'), ((15, 16, 21, 22), u'T0_init')), (((21, 22, 27, 28), u'T0_init'), ((20, 21, 26, 27), u'T0_init')), (((7, 8, 13, 14), u'T0_init'), ((7, 8, 13, 14), u'T0_init')), (((7, 8, 13, 14), u'T0_init'), ((14, 15, 20, 21), u'T0_init')), (((7, 8, 13, 14), u'T0_init'), ((13, 14, 19, 20), u'T0_init')), (((7, 8, 13, 14), u'T0_init'), ((8, 9, 14, 15), u'T0_init')), (((15, 16, 21, 22), u'T0_init'), ((21, 22, 27, 28), u'T0_init')), (((15, 16, 21, 22), u'T0_init'), ((14, 15, 20, 21), u'T0_init')), (((15, 16, 21, 22), u'T0_init'), ((15, 16, 21, 22), u'T0_init')), (((15, 16, 21, 22), u'T0_init'), ((20, 21, 26, 27), u'T0_init')), (((15, 16, 21, 22), u'T0_init'), ((8, 9, 14, 15), u'T0_init')), (((20, 21, 26, 27), u'T0_init'), ((15, 16, 21, 22), u'T0_init')), (((20, 21, 26, 27), u'T0_init'), ((14, 15, 20, 21), u'T0_init')), (((20, 21, 26, 27), u'T0_init'), ((13, 14, 19, 20), u'T0_init')), (((20, 21, 26, 27), u'T0_init'), ((20, 21, 26, 27), u'T0_init')), (((20, 21, 26, 27), u'T0_init'), ((21, 22, 27, 28), u'T0_init')), (((13, 14, 19, 20), u'T0_init'), ((7, 8, 13, 14), u'T0_init')), (((13, 14, 19, 20), u'T0_init'), ((14, 15, 20, 21), u'T0_init')), (((13, 14, 19, 20), u'T0_init'), ((13, 14, 19, 20), u'T0_init')), (((13, 14, 19, 20), u'T0_init'), ((20, 21, 26, 27), u'T0_init')), (((13, 14, 19, 20), u'T0_init'), ((8, 9, 14, 15), u'T0_init')), (((14, 15, 20, 21), u'T0_init'), ((21, 22, 27, 28), u'T0_init')), (((14, 15, 20, 21), u'T0_init'), ((15, 16, 21, 22), u'T0_init')), (((14, 15, 20, 21), u'T0_init'), ((7, 8, 13, 14), u'T0_init')), (((14, 15, 20, 21), u'T0_init'), ((20, 21, 26, 27), u'T0_init')), (((14, 15, 20, 21), u'T0_init'), ((13, 14, 19, 20), u'T0_init')), (((14, 15, 20, 21), u'T0_init'), ((14, 15, 20, 21), u'T0_init')), (((14, 15, 20, 21), u'T0_init'), ((8, 9, 14, 15), u'T0_init')), (((8, 9, 14, 15), u'T0_init'), ((7, 8, 13, 14), u'T0_init')), (((8, 9, 14, 15), u'T0_init'), ((14, 15, 20, 21), u'T0_init')), (((8, 9, 14, 15), u'T0_init'), ((13, 14, 19, 20), u'T0_init')), (((8, 9, 14, 15), u'T0_init'), ((8, 9, 14, 15), u'T0_init')), (((8, 9, 14, 15), u'T0_init'), ((15, 16, 21, 22), u'T0_init'))]

nodes = set([n for e in edges for n in e])
convert = {n:i for i,n in enumerate(nodes)}
edges_converted = [(convert[e[0]],convert[e[1]]) for e in edges]


G = nx.DiGraph()
G.add_edges_from(edges_converted)

print G.edges()
print nx.is_strongly_connected(G)


nx.draw(G)
plt.show()
