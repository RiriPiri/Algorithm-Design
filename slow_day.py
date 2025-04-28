# Slow Day Simulation
import networkx as nx
import matplotlib.pyplot as plt
import heapq
import random
from time import sleep
from threading import Thread, Lock

plt.ion()

G = nx.Graph()
edges = [
    ('Pharmacy', 'ICU', 2),
    ('Pharmacy', 'Ward A', 3),
    ('Ward A', 'Ward B', 4),
    ('Ward B', 'Ward C', 3),
    ('ICU', 'Ward C', 5),
    ('Ward C', 'Operating Room', 6),
    ('ICU', 'Pharmacy', 2),
    ('Ward A', 'ICU', 3)
]
G.add_weighted_edges_from(edges)

# Slow day: Mostly routine, few urgent
orders = [
    {"id": 1, "ward": "Ward A", "urgency": "Routine", "medication": "Vitamin Supplement"},
    {"id": 2, "ward": "Ward B", "urgency": "Routine", "medication": "Daily Medication"},
    {"id": 3, "ward": "Ward C", "urgency": "Urgent", "medication": "Painkiller"},
    {"id": 4, "ward": "ICU", "urgency": "Routine", "medication": "Regular Checkup Kit"}
]
priority_map = {"Emergency": 3, "Urgent": 2, "Routine": 1}

queue = []
for order in orders:
    heapq.heappush(queue, (-priority_map[order["urgency"]], order["id"], order))

robot1_orders = []
robot2_orders = []
for i, (_, _, order) in enumerate(queue):
    if i % 2 == 0:
        robot1_orders.append(order)
    else:
        robot2_orders.append(order)

pos = nx.spring_layout(G)
plot_lock = Lock()

class Robot(Thread):
    def __init__(self, robot_id, current_location, orders):
        super().__init__()
        self.robot_id = robot_id
        self.current_location = current_location
        self.orders = orders
        self.deliveries = []

    def run(self):
        while self.orders:
            order = self.orders.pop(0)
            destination = order['ward']
            medication = order['medication']
            print(f"\nRobot {self.robot_id} delivering {medication} to {destination} (Urgency: {order['urgency']})")

            all_edges = list(G.edges())
            blocked_edge = random.choice(all_edges)
            print(f"Obstacle detected on edge: {blocked_edge}")

            is_large_obstacle = random.choices([True, False], weights=[0.2, 0.8])[0]  # 20% large (rare)
            if is_large_obstacle:
                print("Large obstacle detected! Recalculating path...")
                G_temp = G.copy()
                G_temp.remove_edge(*blocked_edge)

                try:
                    path, total_time = johnson_algorithm(self.current_location, destination, G_temp)
                    print(f"New Path (with obstacle): {path}, Estimated time: {total_time}s")

                    for node in path:
                        draw_graph(node, [blocked_edge])
                        sleep(1)

                    self.current_location = destination
                    self.deliveries.append(order)
                    print(f"Robot {self.robot_id} completed delivery to {destination}!")
                    notify_staff(order)
                except nx.NetworkXNoPath:
                    print("⚠️ No path found due to large obstacle. Robot waits or notifies operator.")
                    draw_graph(self.current_location, [blocked_edge])
                    sleep(2)
            else:
                u, v = blocked_edge
                original_weight = G[u][v]['weight']
                G[u][v]['weight'] = original_weight * 2

                try:
                    path, total_time = johnson_algorithm(self.current_location, destination, G)
                    print(f"Path (with obstacle): {path}, Estimated time: {total_time}s")

                    for node in path:
                        draw_graph(node, [blocked_edge])
                        sleep(1)

                    G[u][v]['weight'] = original_weight
                    self.current_location = destination
                    self.deliveries.append(order)
                    print(f"Robot {self.robot_id} completed delivery to {destination}!")
                    notify_staff(order)
                except nx.NetworkXNoPath:
                    print("⚠️ No path found. Robot waits or notifies operator.")
                    draw_graph(self.current_location, [blocked_edge])
                    sleep(2)

def notify_staff(order):
    print(f"Staff notified: {order['medication']} delivered to {order['ward']}")

def draw_graph(current_node=None, blocked_edges=[]):
    with plot_lock:
        filtered_pos = {node: pos[node] for node in G.nodes if node != 'new_source'}
        colors = ['green' if node == current_node else 'lightblue' for node in G.nodes()]
        plt.clf()
        nx.draw(G, pos=filtered_pos, with_labels=True, node_color=colors, node_size=1000)

        edge_colors = []
        for u, v in G.edges():
            if (u, v) in blocked_edges or (v, u) in blocked_edges:
                edge_colors.append('red')
            else:
                edge_colors.append('black')

        nx.draw_networkx_edges(G, filtered_pos, edge_color=edge_colors)
        nx.draw_networkx_edge_labels(G, filtered_pos, edge_labels={(u, v): d['weight'] for u, v, d in G.edges(data=True)})
        plt.pause(0.1)
        plt.draw()

def johnson_algorithm(source, destination, G):
    new_source = 'new_source'
    G.add_node(new_source)

    for node in G.nodes():
        if node != new_source:
            G.add_edge(new_source, node, weight=0)

    length, path = nx.single_source_bellman_ford(G, new_source)
    G.remove_node(new_source)

    try:
        path = nx.dijkstra_path(G, source, destination, weight='weight')
        total_time = nx.dijkstra_path_length(G, source, destination, weight='weight')
        return path, total_time
    except nx.NetworkXNoPath:
        raise nx.NetworkXNoPath

def close_plot():
    plt.ioff()
    plt.show()

def start_simulation():
    robot1 = Robot(robot_id=1, current_location='Pharmacy', orders=robot1_orders)
    robot2 = Robot(robot_id=2, current_location='Pharmacy', orders=robot2_orders)

    robot1.start()
    robot2.start()

    robot1.join()
    robot2.join()

start_simulation()
close_plot()
