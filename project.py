import heapq
import matplotlib.pyplot as plt
import networkx as nx

def heuristic(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

def a_star_search(graph, start, goal):
    queue = []
    heapq.heappush(queue, (0, start))
    came_from = {}
    cost_so_far = {start: 0}
    
    while queue:
        current_priority, current_node = heapq.heappop(queue)

        if current_node == goal:
            break

        for neighbor in graph.neighbors(current_node):
            new_cost = cost_so_far[current_node] + graph[current_node][neighbor]['weight']
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(graph.nodes[goal]['pos'], graph.nodes[neighbor]['pos'])
                heapq.heappush(queue, (priority, neighbor))
                came_from[neighbor] = current_node

    return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def plot_graph(graph, path=None):
    pos = nx.get_node_attributes(graph, 'pos')
    labels = {node: str(node) for node in graph.nodes}
    nx.draw(graph, pos, with_labels=True, node_color='lightblue', node_size=500)
    nx.draw_networkx_labels(graph, pos, labels)
    edge_labels = nx.get_edge_attributes(graph, 'weight')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels)
    if path:
        edge_path = list(zip(path, path[1:]))
        nx.draw_networkx_edges(graph, pos, edgelist=edge_path, edge_color='red', width=3)
    plt.title("Route Optimization with A*")
    plt.show()

G = nx.Graph()

positions = {
    'A': (0, 0),
    'B': (2, 1),
    'C': (4, 0),
    'D': (6, 1),
    'E': (8, 0),
    'F': (5, 3)
}
for node, pos in positions.items():
    G.add_node(node, pos=pos)

edges = [
    ('A', 'B', 2.2),
    ('B', 'C', 2.2),
    ('C', 'D', 2.2),
    ('D', 'E', 2.2),
    ('B', 'F', 2.0),
    ('F', 'D', 2.0)
]
G.add_weighted_edges_from(edges)

start = 'A'
goal = 'E'
came_from, cost_so_far = a_star_search(G, start, goal)
path = reconstruct_path(came_from, start, goal)

print("Optimal path:", path)
print("Total cost:", cost_so_far[goal])
plot_graph(G, path)