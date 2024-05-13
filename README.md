# Egypt Route Optimization App

This application optimizes routes between cities in Egypt using various search algorithms and displays the results with estimated costs and times. It provides a graphical user interface (GUI) for user interaction.

## Features

- **Graphical User Interface (GUI)**: Allows users to input start and destination cities, choose optimization preferences, and view optimized routes.
- **Route Optimization Algorithms**:
  - A* Search: Finds the shortest path based on a heuristic function (considering gas cost).
  - Depth-First Search (DFS): Attempts to find a path, useful for exploration (with gas cost calculation).
  - Breadth-First Search (BFS): Finds the shortest path in an unweighted graph (with gas cost calculation).
- **Cost and Time Estimation**: Displays total distance, gas cost, and estimated time for each route.
- **Integration with Google Maps**: Users can view optimized routes on Google Maps directly from the application.

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/your-username/egypt-route-optimization-app.git

2. Navigate to the project directory:
    ```bash
    cd egypt-route-optimization-app

3. Install dependencies:
    ```bash
    pip install -r requirements.txt

4. Run the application:
    ```bash
    python route_optimization.py

