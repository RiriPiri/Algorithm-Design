import networkx as nx
import matplotlib.pyplot as plt
import heapq
import random
from time import sleep
from threading import Thread, Lock

plt.ion()

# Create hospital map
G = nx.DiGraph()
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

# Orders (medicines to deliver)
orders = [
    {"id": 1, "ward": "ICU", "urgency": "Emergency", "medication": "Painkiller"},
    {"id": 2, "ward": "Ward A", "urgency": "Routine", "medication": "Antibiotics"},
    {"id": 3, "ward": "Ward B", "urgency": "Urgent", "medication": "Sedative"},
    {"id": 4, "ward": "Operating Room", "urgency": "Emergency", "medication": "Anesthesia"}
]
priority_map = {"Emergency": 3, "Urgent": 2, "Routine": 1}

# Priority Queue
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

# Heuristic function for A*
def heuristic(u, v):
    # Simple heuristic: constant small value
    return 1

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

            is_large_obstacle = random.random() < 0.5  # 50% chance large

            if is_large_obstacle:
                print("Large obstacle detected! Using Johnson's to recalculate all paths.")
                G_temp = G.copy()
                G_temp.remove_edge(*blocked_edge)
                try:
                    all_pairs_paths = nx.johnson(G_temp, weight='weight')

                    if self.current_location in all_pairs_paths and destination in all_pairs_paths[self.current_location]:
                        path = all_pairs_paths[self.current_location][destination]
                        total_time = sum(G_temp[u][v]['weight'] for u, v in zip(path[:-1], path[1:]))
                        print(f"New Johnson's path: {path}, Estimated time: {total_time}s")

                        for node in path:
                            draw_graph(node, [blocked_edge])
                            sleep(1)

                        self.current_location = destination
                        self.deliveries.append(order)
                        print(f"Robot {self.robot_id} completed delivery to {destination}!")
                        notify_staff(order)
                    else:
                        print("⚠️ No Johnson's path found. Robot waits.")
                        draw_graph(self.current_location, [blocked_edge])
                        sleep(2)
                except nx.NetworkXError as e:
                    print("⚠️ Johnson's failed:", e)
                    draw_graph(self.current_location, [blocked_edge])
                    sleep(2)
            else:
                try:
                    path = nx.astar_path(G, self.current_location, destination, heuristic=heuristic, weight='weight')
                    total_time = sum(G[u][v]['weight'] for u, v in zip(path[:-1], path[1:]))
                    print(f"A* Path: {path}, Estimated time: {total_time}s")

                    for node in path:
                        draw_graph(node, [blocked_edge])
                        sleep(1)

                    self.current_location = destination
                    self.deliveries.append(order)
                    print(f"Robot {self.robot_id} completed delivery to {destination}!")
                    notify_staff(order)
                except nx.NetworkXNoPath:
                    print("⚠️ No A* path found. Robot waits.")
                    draw_graph(self.current_location, [blocked_edge])
                    sleep(2)

def notify_staff(order):
    print(f"Staff notified: {order['medication']} delivered to {order['ward']}")

def draw_graph(current_node=None, blocked_edges=[]):
    with plot_lock:
        plt.clf()
        colors = ['green' if node == current_node else 'lightblue' for node in G.nodes()]
        nx.draw(G, pos, with_labels=True, node_color=colors, node_size=1000)

        edge_colors = []
        for u, v in G.edges():
            if (u, v) in blocked_edges or (v, u) in blocked_edges:
                edge_colors.append('red')
            else:
                edge_colors.append('black')

        nx.draw_networkx_edges(G, pos, edge_color=edge_colors)
        nx.draw_networkx_edge_labels(G, pos, edge_labels={(u, v): d['weight'] for u, v, d in G.edges(data=True)})
        plt.pause(0.1)
        plt.draw()

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
