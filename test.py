import networkx as nx
import matplotlib.pyplot as plt 

G = nx.MultiDiGraph()

G.add_edges_from([
    (1, 2),
    (3, 2)
])

plt.figure(figsize=(8,8))
nx.draw(G)

print nx.has_path(G,1,3)
print nx.has_path(G,3,1)
plt.show()