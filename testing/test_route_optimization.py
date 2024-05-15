import unittest
from route_optimization import RouteOptimizationApp, GraphUtility

class TestRouteOptimizationApp(unittest.TestCase):

    def setUp(self):
        # Initialize test instances
        self.app = RouteOptimizationApp()
        self.graph_utility = GraphUtility()

    def test_valid_route_optimization(self):
        # Test valid route optimization functionality
        start_city = "Cairo"
        dest_city = "Luxor"
        preference = "Optimal Solution"
        expected_distance = 700  # Expected distance from Cairo to Luxor
        expected_gas_cost = (expected_distance / 7) * 10  # Expected gas cost for the route

        # Simulate user input and route finding
        self.app.start_entry.insert(0, start_city)
        self.app.dest_entry.insert(0, dest_city)
        self.app.preference_var.set(preference)
        self.app.find_route()

        # Retrieve actual route details from message box
        message = self.app.tk._test_msg
        actual_distance = float(message.split("Total Distance: ")[1].split(" km")[0])
        actual_gas_cost = float(message.split("Total Gas Cost: ")[1].split(" EGP")[0])

        # Assert expected route details match actual results
        self.assertAlmostEqual(actual_distance, expected_distance, delta=0.1)  # Check distance with delta for precision
        self.assertAlmostEqual(actual_gas_cost, expected_gas_cost, delta=0.1)  # Check gas cost with delta for precision

    def test_invalid_preference(self):
        # Test handling of invalid route optimization preference
        invalid_preference = "Invalid Preference"
        expected_error_message = "Invalid preference selection."

        # Simulate user input with invalid preference
        self.app.preference_var.set(invalid_preference)
        with self.assertRaisesRegex(ValueError, expected_error_message):
            self.app.find_route()

    def test_graph_loading(self):
        # Test loading of graph data from JSON file
        expected_num_nodes = 27  # Expected number of cities in Egypt
        expected_num_edges = 60  # Expected number of connections (edges) between cities

        # Load graph data
        graph = self.graph_utility.load_graph_from_json("egypt_map.json")

        # Assert expected number of nodes and edges
        self.assertEqual(len(graph.nodes()), expected_num_nodes)
        self.assertEqual(len(graph.edges()), expected_num_edges)

    def test_calculate_route_details(self):
        # Test calculation of route details (distance, gas cost, time)
        path = ["Cairo", "Luxor", "Aswan"]
        expected_distance = 1100  # Expected total distance for the route
        expected_gas_cost = (expected_distance / 7) * 10  # Expected gas cost for the route

        # Calculate route details
        details = self.graph_utility.calculate_route_details(path)

        # Assert expected route details
        self.assertEqual(details["total_distance"], expected_distance)
        self.assertAlmostEqual(details["total_gas_cost"], expected_gas_cost, delta=0.1)  # Check gas cost with delta

if __name__ == "__main__":
    unittest.main()
