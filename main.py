import heapq
import matplotlib.pyplot as plt
import networkx as nx
import time

class Graph():
    def __init__(self):
        self.graph = {}

    def add_node(self, node):
        if node not in self.graph:
            self.graph[node] = []

    def remove_node(self, node):
        if node in self.graph:
            del self.graph[node]
            for neighbors in self.graph.values():
                neighbors[:] = [edge for edge in neighbors if edge[0] != node]

    def add_edge(self, u, v, weight):
        if u not in self.graph:
            self.add_node(u)
        if v not in self.graph:
            self.add_node(v)
        self.graph[u].append((v, weight))
        self.graph[v].append((u, weight))

    def remove_edge(self, u, v):
        if u in self.graph:
            self.graph[u] = [edge for edge in self.graph[u] if edge[0] != v]
        if v in self.graph:
            self.graph[v] = [edge for edge in self.graph[v] if edge[0] != u]

    def input_graph(self, adjacency_list):
        self.graph = adjacency_list

    def print_solution(self, dist, path, src):
        print("Vertex \t Distance from Source \t Path")
        for node in range(len(dist)):
            if dist[node] != float('inf'):
                print(f"{node} \t\t {dist[node]} \t\t {self.construct_path(path, src, node)}")

    def construct_path(self, path, src, target):
        result_path = []
        current = target
        while current is not None:
            result_path.append(current)
            current = path[current]
        return list(reversed(result_path))

    def dijkstra(self, src):
        start_time = time.time()

        dist = {node: float('inf') for node in self.graph}
        path = {node: None for node in self.graph}
        dist[src] = 0

        pq = [(0, src)]

        while pq:
            current_dist, u = heapq.heappop(pq)

            if current_dist > dist[u]:
                continue

            for neighbor, weight in self.graph[u]:
                if dist[u] + weight < dist[neighbor]:
                    dist[neighbor] = dist[u] + weight
                    path[neighbor] = u
                    heapq.heappush(pq, (dist[neighbor], neighbor))

        end_time = time.time()

        self.print_solution(dist, path, src)
        print(f"Time taken to compute: {end_time - start_time:.6f} seconds")

        return dist, path

    def visualize_graph(self):
        G = nx.Graph()
        for node, neighbors in self.graph.items():
            for neighbor, weight in neighbors:
                G.add_edge(node, neighbor, weight=weight)

        pos = nx.spring_layout(G)
        edge_labels = nx.get_edge_attributes(G, 'weight')
        nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10)
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
        plt.title("Graph Visualization")
        plt.show()

g = Graph()

g.input_graph({
    0: [(1, 4), (7, 8)],
    1: [(0, 4), (2, 8), (7, 11)],
    2: [(1, 8), (3, 7), (8, 2), (5, 4)],
    3: [(2, 7), (4, 9), (5, 14)],
    4: [(3, 9), (5, 10)],
    5: [(2, 4), (3, 14), (4, 10), (6, 2)],
    6: [(5, 2), (7, 1), (8, 6)],
    7: [(0, 8), (1, 11), (6, 1), (8, 7)],
    8: [(2, 2), (6, 6), (7, 7)]
})

g.add_edge(8, 3, 5)
g.remove_edge(0, 1)
g.add_node(9)
g.add_edge(9, 8, 3)
g.remove_node(7)

g.visualize_graph()

g.dijkstra(0)
