from collections import defaultdict

import networkx as nx
from networkx.utils import *
from networkx.algorithms.traversal.edgedfs import helper_funcs, edge_dfs


def find_cycle(G, source=None, orientation='original'):
    out_edge, key, tailhead = helper_funcs(G, orientation)

    explored = set()
    cycle = []
    final_node = None
    for start_node in G.nbunch_iter(source):

        if start_node in explored:
            # No loop is possible.
            continue

        edges = []
        # All nodes seen in this iteration of edge_dfs
        seen = {start_node}
        # Nodes in active path.
        active_nodes = {start_node}
        previous_node = None
        for edge in edge_dfs(G, start_node, orientation):
            # Determine if this edge is a continuation of the active path.
            tail, head = tailhead(edge)
            if previous_node is not None and tail != previous_node:
                # This edge results from backtracking.
                # Pop until we get a node whose head equals the current tail.
                # So for example, we might have:
                #  (0,1), (1,2), (2,3), (1,4)
                # which must become:
                #  (0,1), (1,4)
                while True:
                    try:
                        popped_edge = edges.pop()
                    except IndexError:
                        edges = []
                        active_nodes = {tail}
                        break
                    else:
                        popped_head = tailhead(popped_edge)[1]
                        active_nodes.remove(popped_head)

                    if edges:
                        last_head = tailhead(edges[-1])[1]
                        if tail == last_head:
                            break

            edges.append(edge)

            if head in active_nodes:
                # We have a loop!
                cycle.extend(edges)
                final_node = head
                break
            elif head in explored:
                # Then we've already explored it. No loop is possible.
                break
            else:
                seen.add(head)
                active_nodes.add(head)
                previous_node = head

        if cycle:
            break
        else:
            explored.update(seen)

    else:
        assert(len(cycle) == 0)
        raise nx.exception.NetworkXNoCycle('No cycle found.')

    # We now have a list of edges which ends on a cycle.
    # So we need to remove from the beginning edges that are not relevant.

    for i, edge in enumerate(cycle):
        tail, head = tailhead(edge)
        if tail == final_node:
            break
    return cycle

G=nx.DiGraph()

G.add_edges_from([(0,1),(1,2),(1,1),(0,0)])

while True:
    u = find_cycle(G)
    G.remove_edge(*u[-1])
    print u
