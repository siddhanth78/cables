from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import json
import numpy as np
import matplotlib.pyplot as plt
import osmnx as ox
import networkx as nx
import pandas as pd
from shapely.geometry import LineString
import geopandas as gpd
import time
import os

def create_data_model():
    """Stores the data for the problem."""
    with open("df_dist.json", "r") as file:
        df = json.loads(file.read())
    data = {}
    dist_matrix = []
    for d in df:
        dist_matrix.append(df[d])
    data["distance_matrix"] = dist_matrix
    data["num_vehicles"] = 8
    data["depot"] = 0
    with open("code_map.json", "r") as filec:
        locations = json.loads(filec.read())
    coords = []
    for l in locations:
        coords.append(locations[l])
    data["locations"] = coords
    print("Loaded locations:", locations)
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    max_route_distance = 0
    routes = []
    
    for vehicle_id in range(data["num_vehicles"]):
        if not routing.IsVehicleUsed(solution, vehicle_id):
            continue
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route = []
        
        while not routing.IsEnd(index):
            node_idx = manager.IndexToNode(index)
            route.append(node_idx)
            plan_output += f" {node_idx} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        
        # Add the last node
        node_idx = manager.IndexToNode(index)
        route.append(node_idx)
        plan_output += f"{node_idx}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        print(plan_output)
        
        max_route_distance = max(route_distance, max_route_distance)
        routes.append(route)
        
    print(f"Maximum of the route distances: {max_route_distance}m")
    return routes

def download_road_network(data):
    """Download the road network for the area containing all locations."""
    # Extract coordinates for all locations
    lats = [loc[2] for loc in data["locations"]]  # Y coordinate
    lons = [loc[1] for loc in data["locations"]]  # X coordinate
    
    # Add a small buffer around the area to ensure all locations are included
    north = max(lats) + 0.02
    south = min(lats) - 0.02
    east = max(lons) + 0.02
    west = min(lons) - 0.02
    
    print(f"Downloading road network for area: N={north:.4f}, S={south:.4f}, E={east:.4f}, W={west:.4f}")
    
    try:
        # Download the street network
        G = ox.graph_from_bbox(north, south, east, west, network_type='drive')
        print(f"Successfully downloaded network with {len(G.nodes)} nodes and {len(G.edges)} edges")
        return G
    except Exception as e:
        print(f"Error downloading road network: {e}")
        print("Falling back to simplified network...")
        # Create a simplified network by connecting all points directly
        G = nx.DiGraph()
        for i, loc in enumerate(data["locations"]):
            G.add_node(i, x=loc[1], y=loc[2])
        
        # Connect all nodes to each other
        for i in range(len(data["locations"])):
            for j in range(len(data["locations"])):
                if i != j:
                    dist = data["distance_matrix"][i][j]
                    G.add_edge(i, j, length=dist)
        
        return G

def get_nearest_node(G, point):
    """Get the nearest node in the graph to the given point."""
    try:
        # Try to use OSMnx's function if it's a proper OSMnx graph
        return ox.distance.nearest_nodes(G, point[0], point[1])
    except:
        # Simplified approach for our backup graph
        nodes = list(G.nodes(data=True))
        closest_node = None
        min_dist = float('inf')
        
        for node, data in nodes:
            dist = ((data['x'] - point[0])**2 + (data['y'] - point[1])**2)**0.5
            if dist < min_dist:
                min_dist = dist
                closest_node = node
                
        return closest_node

def find_shortest_paths(G, data, routes):
    """Find the shortest paths on the road network for each route."""
    detailed_routes = []
    
    is_osmnx_graph = hasattr(G, "graph") and "crs" in G.graph
    
    for vehicle_id, route in enumerate(routes):
        detailed_route = []
        
        for i in range(len(route) - 1):
            from_idx = route[i]
            to_idx = route[i + 1]
            
            # Get the coordinates of the locations
            from_coords = (data["locations"][from_idx][1], data["locations"][from_idx][2])  # lon, lat
            to_coords = (data["locations"][to_idx][1], data["locations"][to_idx][2])        # lon, lat
            
            if is_osmnx_graph:
                # Get the nearest network nodes to the points
                from_node = ox.distance.nearest_nodes(G, from_coords[0], from_coords[1])
                to_node = ox.distance.nearest_nodes(G, to_coords[0], to_coords[1])
                
                try:
                    # Find the shortest path
                    path = nx.shortest_path(G, from_node, to_node, weight='length')
                    
                    # Get coordinates for all nodes in the path
                    path_coords = []
                    for node in path:
                        coords = (G.nodes[node]['x'], G.nodes[node]['y'])
                        path_coords.append(coords)
                    
                    detailed_route.append({
                        'from_idx': from_idx,
                        'to_idx': to_idx,
                        'path': path,
                        'path_coords': path_coords
                    })
                    
                except nx.NetworkXNoPath:
                    print(f"No path found between {from_idx} and {to_idx}, using direct line")
                    detailed_route.append({
                        'from_idx': from_idx,
                        'to_idx': to_idx,
                        'path': [from_idx, to_idx],
                        'path_coords': [from_coords, to_coords]
                    })
            else:
                # For simplified graph, just use direct connections
                detailed_route.append({
                    'from_idx': from_idx,
                    'to_idx': to_idx,
                    'path': [from_idx, to_idx],
                    'path_coords': [from_coords, to_coords]
                })
                
        detailed_routes.append(detailed_route)
    
    return detailed_routes

def plot_routes_on_map(G, data, detailed_routes):
    """Plot the routes on a map with the actual road network."""
    is_osmnx_graph = hasattr(G, "graph") and "crs" in G.graph
    
    # Create figure
    fig, ax = plt.subplots(figsize=(15, 12))
    
    if is_osmnx_graph:
        # Plot the base map
        ox.plot_graph(G, ax=ax, node_size=0, edge_linewidth=0.5, edge_color='#999999', show=False)
    else:
        # Plot a simplified base map
        for u, v, data in G.edges(data=True):
            u_x, u_y = G.nodes[u]['x'], G.nodes[u]['y']
            v_x, v_y = G.nodes[v]['x'], G.nodes[v]['y']
            ax.plot([u_x, v_x], [u_y, v_y], color='#CCCCCC', linewidth=0.5, alpha=0.5)
    
    # Generate colors for each route
    colors = plt.cm.viridis(np.linspace(0, 1, data["num_vehicles"]))
    
    # Plot all locations
    for i, loc in enumerate(data["locations"]):
        x, y = loc[1], loc[2]
        if i == data["depot"]:
            # Depot
            ax.scatter(x, y, c='red', marker='*', s=200, zorder=10, edgecolor='black')
            ax.annotate(f"Depot: {loc[0]}", (x, y), xytext=(5, 5), 
                        textcoords='offset points', fontsize=12, weight='bold')
        else:
            # Regular location
            ax.scatter(x, y, c='blue', marker='o', s=100, zorder=10, edgecolor='black')
            ax.annotate(f"{loc[0]}", (x, y), xytext=(5, 5), 
                        textcoords='offset points', fontsize=10)
    
    # Plot each route
    for vehicle_id, route in enumerate(detailed_routes):
        color = colors[vehicle_id % len(colors)]
        
        for segment in route:
            path_coords = segment['path_coords']
            
            # Plot the actual route
            x_coords = [coord[0] for coord in path_coords]
            y_coords = [coord[1] for coord in path_coords]
            
            ax.plot(x_coords, y_coords, '-', color=color, linewidth=2.5, 
                   label=f'Vehicle {vehicle_id+1}' if segment == route[0] else "", 
                   alpha=0.8, zorder=5)
    
    # Create the legend with only one entry per vehicle
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='best')
    
    # Set labels and title
    plt.title(f"Cable Routes with Real Roads - {data['num_vehicles']} Rings", fontsize=16)
    plt.grid(True, linestyle='--', alpha=0.3)
    
    # No need to set axis equal for geographic coordinates
    plt.tight_layout()
    
    # Save the figure
    plt.savefig("vrp_real_routes_map.png", dpi=300, bbox_inches='tight')
    print("Map saved as 'vrp_real_routes_map.png'")
    
    # Show the plot
    plt.show()

def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        55000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    print("Solving VRP problem...")
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console and get the routes
    if solution:
        routes = print_solution(data, manager, routing, solution)
        
        print("\nDownloading road network (this may take a while)...")
        G = download_road_network(data)
        
        print("Finding detailed routes on road network...")
        detailed_routes = find_shortest_paths(G, data, routes)
        
        print("Plotting routes on map...")
        plot_routes_on_map(G, data, detailed_routes)
    else:
        print("No solution found!")

if __name__ == "__main__":
    main()
