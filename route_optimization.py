""" Main file Contain all Functionality of the Program """
import tkinter as tk
from tkinter import messagebox, ttk
import heapq
import json
import networkx as nx
import requests


class RouteOptimizationApp(tk.Tk):
    """ The GUI of the Program Control"""
    def __init__(self):
        super().__init__()
        self.title("Egypt Route Optimization App")
        self.geometry("800x600")

        # Background Control
        #self.background_image = tk.PhotoImage(file="background_image.png")
        self.background_label = tk.Label(self, image=self.background_image)
        self.background_label.place(x=0, y=0, relwidth=1, relheight=1)

        self.start_label = tk.Label(self, text="Start City:")
        self.start_entry = tk.Entry(self, width=30)
        self.dest_label = tk.Label(self, text="Destination City:")
        self.dest_entry = tk.Entry(self, width=30)
        self.preference_label = tk.Label(self, text="Preference:")
        self.preference_var = tk.StringVar(self)
        self.preference_var.set("Optimal Solution")
        self.preference_option = ttk.Combobox(self, textvariable=self.preference_var,
                        values=
                                        ["Optimal Solution", "The Known Way", "The Direct Way"])

        self.route_button = tk.Button(self, text="Find Route", command=self.find_route)

        self.start_label.pack(pady=20)
        self.start_entry.pack(pady=5)
        self.dest_label.pack(pady=20)
        self.dest_entry.pack(pady=5)
        self.preference_label.pack(pady=20)
        self.preference_option.pack(pady=5)
        self.route_button.pack(pady=20)

        # Load Egypt map from JSON file
        with open('egypt_map.json', 'r',encoding='UTF-8') as f:
            self.graph = nx.Graph(json.load(f))

    def find_route(self):
        """ Find Route Function """
        start = self.start_entry.get().strip().capitalize()
        dest = self.dest_entry.get().strip().capitalize()
        preference = self.preference_var.get()

        if not start or not dest:
            messagebox.showerror("Error", "Please enter both start and destination cities.")
            return

        if start not in self.graph.nodes or dest not in self.graph.nodes:
            messagebox.showerror("Error", "Invalid start or destination city.")
            return

        try:
            if preference == "Optimal Solution":
                path, details = self.a_star_search(start, dest)
            elif preference == "The Known Way":
                path, details = self.dfs_search(start, dest)
            elif preference == "The Direct Way":
                path, details = self.bfs_search(start, dest)
            else:
                messagebox.showerror("Error", "Invalid preference selection.")
                return

            # Display optimized route and details in pop-up message box
            message = f"Optimized route: {' -> '.join(path)}\n"
            message += f"Total Distance: {details['total_distance']} km\n"
            message += f"Total Gas Cost: {details['total_gas_cost']:.2f} EGP\n"
            message += f"Estimated Time: {details['total_time']:.2f} hours"
            messagebox.showinfo("Route Details", message)

            # Save detailed route information to TXT file
            self.save_route_to_file(start, dest, path, details)

            # Display route on Google Maps
            self.display_google_maps(path)
        except nx.NetworkXNoPath:
            messagebox.showerror("Error", "No path found between selected cities.")

    def a_star_search(self, start, goal):
        """ A* search implementation with gas cost heuristic function """ 
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while frontier:
            current_cost, current_node = heapq.heappop(frontier)

            if current_node == goal:
                break

            for next_node in self.graph.neighbors(current_node):
                new_cost = cost_so_far[current_node] + self.graph[current_node][next_node]['distance']
                # Heuristic function based on gas cost and remaining distance
                gas_cost = new_cost / 7 * 10  # Assuming 1 liter of gas for every 7 km costing 10 pounds
                priority = new_cost + gas_cost

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current_node

        # Reconstruct path and calculate details
        path = []
        node = goal
        while node != start:
            path.append(node)
            node = came_from[node]
        path.append(start)
        path.reverse()

        # Calculate route details (total distance, gas cost, time estimate)
        route_details = self.calculate_route_details(path)
        return path, route_details

    def dfs_search(self, start, goal):
        # Depth-first search (DFS) implementation
        stack = [(start, [start])]
        visited = set()

        while stack:
            current_node, path = stack.pop()

            if current_node == goal:
                # Calculate route details for DFS (assuming no gas cost calculation)
                route_details = self.calculate_route_details(path)
                return path, route_details

            if current_node not in visited:
                visited.add(current_node)
                for neighbor in self.graph.neighbors(current_node):
                    if neighbor not in visited:
                        stack.append((neighbor, path + [neighbor]))

        raise nx.NetworkXNoPath

    def bfs_search(self, start, goal):
        # Breadth-first search (BFS) implementation
        queue = [(start, [start])]
        visited = set()

        while queue:
            current_node, path = queue.pop(0)

            if current_node == goal:
                # Calculate route details for BFS (assuming no gas cost calculation)
                route_details = self.calculate_route_details(path)
                return path, route_details

            if current_node not in visited:
                visited.add(current_node)
                for neighbor in self.graph.neighbors(current_node):
                    if neighbor not in visited:
                        queue.append((neighbor, path + [neighbor]))

        raise nx.NetworkXNoPath

    def calculate_route_details(self, path):
        total_distance = 0
        total_gas_cost = 0

        for i in range(len(path) - 1):
            city1 = path[i]
            city2 = path[i + 1]
            distance = self.graph[city1][city2]['distance']
            total_distance += distance

        # Calculate gas cost based on total distance
        total_gas_cost = (total_distance / 7) * 10  # Assuming 1 liter of gas for every 7 km costing 10 pounds

        # Estimate time based on terrain type (not implemented, placeholder)
        total_time = total_distance / 60  # Assuming average speed of 60 km/h

        return {
            "total_distance": total_distance,
            "total_gas_cost": total_gas_cost,
            "total_time": total_time
        }

    def save_route_to_file(self, start, dest, path, details):
        with open('route_solutions.txt', 'a',encoding='UTF-8') as f:
            f.write(f"Route from {start} to {dest}: {' -> '.join(path)}\n")
            f.write(f"Total Distance: {details['total_distance']} km\n")
            f.write(f"Total Gas Cost: {details['total_gas_cost']} EGP\n")
            f.write(f"Total Time: {details['total_time']} hours\n\n")

    def display_google_maps(self, path):
        url = f"https://www.google.com/maps/dir/{'/'.join(path)}"
        response = requests.get(url,timeout=10)
        if response.status_code == 200:
            messagebox.showinfo("Google Maps", f"View the route on Google Maps:\n{url}")
            # Open the route in the default web browser (optional)
            import webbrowser
            webbrowser.open(url)
        else:
            messagebox.showwarning("Google Maps", "Failed to access Google Maps.")

if __name__ == "__main__":
    app = RouteOptimizationApp()
    app.mainloop()
