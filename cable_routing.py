from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import json
import numpy as np

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
    print(locations)
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    max_route_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        if not routing.IsVehicleUsed(solution, vehicle_id):
            continue
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print(f"Maximum of the route distances: {max_route_distance}m")



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
        100000,  # vehicle maximum travel distance
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
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
        plot_vrp_solution(data, manager, routing, solution)
    else:
        print("No solution found !")

import matplotlib.pyplot as plt

def plot_vrp_solution(data, manager, routing, solution):
    """Plots the VRP solution with coordinates scaled by 1/1000."""
    num_locations = len(data['locations'])
    
    # Create figure with appropriate size
    plt.figure(figsize=(12, 10))
    
    # Generate distinct colors for each vehicle route
    colors = plt.cm.viridis(np.linspace(0, 1, data['num_vehicles']))
    
    # Plot locations
    for i in range(num_locations):
        # Scale the coordinates
        x = data['locations'][i][1] 
        y = data['locations'][i][2] 
        
        if i == data['depot']:
            # Plot depot as a red star with larger size
            plt.scatter(x, y, c='red', marker='*', s=200, 
                       label=f"Depot: {data['locations'][i][0]}")
        else:
            # Plot other locations as blue circles
            plt.scatter(x, y, c='blue', marker='o', s=100,
                       label=data['locations'][i][0] if i == 1 else "")
            
            # Optionally add location labels
            plt.annotate(data['locations'][i][0], (x, y), 
                        xytext=(5, 5), textcoords='offset points', fontsize=8)
    
    # Plot routes for each vehicle with different colors
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        points_x = []
        points_y = []
        
        # Start at depot
        node = manager.IndexToNode(index)
        points_x.append(data['locations'][node][1] )
        points_y.append(data['locations'][node][2] )
        
        # Add all stops in the route
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            
            node = manager.IndexToNode(index)
            x = data['locations'][node][1] 
            y = data['locations'][node][2] 
            
            points_x.append(x)
            points_y.append(y)
        
        # Plot the complete route with vehicle-specific color
        plt.plot(points_x, points_y, '-', color=colors[vehicle_id], linewidth=2, 
                label=f'Vehicle {vehicle_id+1}')
    
    # Set labels and title
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title(f"Cable Routes - {data["num_vehicles"]} rings")
    
    # Add grid
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # Make sure axes have equal scale
    plt.axis('equal')
    
    # Use tight layout for better spacing
    plt.tight_layout()
    
    # Show the plot
    plt.show() 

if __name__ == "__main__":
    main()
