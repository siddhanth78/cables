import os
import numpy as np
import geopandas as gpd
import networkx as nx
import folium
from folium.plugins import MarkerCluster
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from shapely.geometry import Point
import matplotlib.pyplot as plt
import pandas as pd
from geopy.distance import geodesic
import random
import argparse

def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Vehicle Routing with Shapefile')
    parser.add_argument('--shapefile', type=str, required=True, 
                        help='Path to the shapefile')
    parser.add_argument('--num_vehicles', type=int, default=3,
                        help='Number of vehicles to use for routing')
    parser.add_argument('--depot_index', type=int, default=0,
                        help='Index of the point to use as depot')
    parser.add_argument('--output', type=str, default='route_map.html',
                        help='Output HTML file for the Folium map')
    parser.add_argument('--visualize_only', action='store_true',
                        help='Only visualize the shapefile without routing')
    return parser.parse_args()

def load_shapefile(shapefile_path):
    """Load shapefile into a GeoDataFrame."""
    if not os.path.exists(shapefile_path):
        raise FileNotFoundError(f"Shapefile not found: {shapefile_path}")
    
    print(f"Loading shapefile from: {shapefile_path}")
    gdf = gpd.read_file(shapefile_path)
    print(f"Loaded shapefile with {len(gdf)} features")
    print(f"Coordinate reference system: {gdf.crs}")
    
    # Ensure we're working with geographic coordinates (lat/lon)
    if gdf.crs and not gdf.crs.is_geographic:
        gdf = gdf.to_crs(epsg=4326)
        print("Converted to geographic coordinates (EPSG:4326)")
    
    return gdf

def extract_points(gdf):
    """Extract points from the GeoDataFrame for routing."""
    points = []
    point_sources = []  # Track where each point came from
    
    # Handle different geometry types
    for idx, row in gdf.iterrows():
        geom = row.geometry
        
        if geom.type == 'Point':
            points.append((geom.y, geom.x))  # (lat, lon)
            source_info = f"Point {idx}"
            if 'name' in row and row['name']:
                source_info += f": {row['name']}"
            point_sources.append(source_info)
        
        elif geom.type == 'MultiPoint':
            for i, point in enumerate(geom.geoms):
                points.append((point.y, point.x))
                source_info = f"MultiPoint {idx}.{i}"
                if 'name' in row and row['name']:
                    source_info += f": {row['name']}"
                point_sources.append(source_info)
        
        elif geom.type in ['LineString', 'MultiLineString']:
            # Use the centroid of lines
            centroid = geom.centroid
            points.append((centroid.y, centroid.x))
            source_info = f"Line {idx}"
            if 'name' in row and row['name']:
                source_info += f": {row['name']}"
            point_sources.append(source_info)
        
        elif geom.type in ['Polygon', 'MultiPolygon']:
            # Use the centroid of polygons
            centroid = geom.centroid
            points.append((centroid.y, centroid.x))
            source_info = f"Polygon {idx}"
            if 'name' in row and row['name']:
                source_info += f": {row['name']}"
            point_sources.append(source_info)
    
    # Create a GeoDataFrame with the extracted points for visualization
    points_df = pd.DataFrame({
        'lat': [p[0] for p in points],
        'lon': [p[1] for p in points],
        'source': point_sources
    })
    
    print(f"Extracted {len(points)} points for routing")
    return points, points_df

def create_distance_matrix(points):
    """Create distance matrix between all points using geodesic distance."""
    size = len(points)
    matrix = np.zeros((size, size))
    
    # Print dimensions for debugging
    print(f"Creating distance matrix for {size} points")
    
    for i in range(size):
        for j in range(size):
            if i != j:
                # Calculate distance in meters
                dist = int(geodesic(points[i], points[j]).meters)
                matrix[i][j] = dist
                # Add some random noise to break ties and encourage multiple routes
                if i != 0 and j != 0:  # Don't add noise to depot distances
                    matrix[i][j] += random.randint(0, 10)
    
    # Verify matrix contains sensible values
    print(f"Distance matrix min value: {np.min(matrix[np.nonzero(matrix)])}")
    print(f"Distance matrix max value: {np.max(matrix)}")
    print(f"Distance matrix mean value: {np.mean(matrix[np.nonzero(matrix)])}")
    
    return matrix.astype(int)

def solve_vrp(distance_matrix, num_vehicles, depot):
    """Solve the Vehicle Routing Problem using Google OR-Tools."""
    # Create the routing model
    manager = pywrapcp.RoutingIndexManager(
        len(distance_matrix), num_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)
    
    # Define the distance callback
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    
    # Set the cost of travel
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # Add capacity constraint - each vehicle should serve approximately equal number of nodes
    def demand_callback(from_index):
        """Return demand of the node."""
        # Each node has a demand of 1, except depot
        from_node = manager.IndexToNode(from_index)
        return 1 if from_node != depot else 0
    
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    
    # Add Capacity constraint to limit nodes per vehicle and force distribution
    nodes_per_vehicle = max(1, (len(distance_matrix) - 1) // num_vehicles)
    vehicle_capacity = nodes_per_vehicle + 1  # Allow some flexibility
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        [vehicle_capacity] * num_vehicles,  # vehicle capacity array
        True,  # start cumul to zero
        'Capacity')
    
    # Add distance dimension
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000000,  # large upper bound
        True,  # start cumul to zero
        'Distance')
    distance_dimension = routing.GetDimensionOrDie('Distance')
    
    # Try to minimize the maximum distance among all vehicles
    distance_dimension.SetGlobalSpanCostCoefficient(100)
    
    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(30)  # 30 seconds time limit
    
    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)
    
    # Process the solution
    routes = []
    if solution:
        max_route_distance = 0
        for vehicle_id in range(num_vehicles):
            route = []
            index = routing.Start(vehicle_id)
            route_distance = 0
            
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route.append(node_index)
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            
            node_index = manager.IndexToNode(index)
            route.append(node_index)
            
            # Only add non-empty routes (more than just depot-to-depot)
            if len(route) > 2 or (len(route) == 2 and route[0] != route[1]):
                routes.append(route)
                max_route_distance = max(route_distance, max_route_distance)
                print(f"Route {vehicle_id}: Distance {route_distance} meters, Stops: {len(route)-2}")
        
        print(f"Found solution with {len(routes)} routes")
        print(f"Maximum route distance: {max_route_distance} meters")
        return routes
    else:
        print("No solution found!")
        return None

def create_folium_map(points, routes, depot_index, gdf=None):
    """Create a Folium map with the optimized routes and original shapefile visualized."""
    # Unpack points if it's a tuple with points_df
    if isinstance(points, tuple):
        points_data = points[1]  # DataFrame with point info
        points = points[0]       # The actual points
    else:
        points_data = None
    
    # Create map centered at the first point
    center_lat, center_lon = points[depot_index]
    m = folium.Map(location=[center_lat, center_lon], zoom_start=12)
    
    # Add shapefile geometries if provided
    if gdf is not None:
        # Add shapefile features to the map
        folium.GeoJson(
            gdf,
            name="Shapefile Features",
            style_function=lambda x: {
                'fillColor': '#ffff00',
                'color': '#000000',
                'weight': 1,
                'fillOpacity': 0.1
            },
            tooltip=folium.GeoJsonTooltip(
                fields=gdf.columns[:5].tolist() if len(gdf.columns) > 0 else [],
                aliases=gdf.columns[:5].tolist() if len(gdf.columns) > 0 else [],
                localize=True
            )
        ).add_to(m)
    
    # Add depot marker
    folium.Marker(
        location=[points[depot_index][0], points[depot_index][1]],
        popup="Depot",
        icon=folium.Icon(color='red', icon='home'),
    ).add_to(m)

    # Load Excel
    locdf = pd.read_excel("SaiyanBlock.xlsx")

    # Add markers
    for index, row in locdf.iterrows():
        folium.Marker(
            location=(row['Lat'], row['Long']),
            popup=row['Name'],
        ).add_to(m)
    
    # Define colors for different routes
    colors = ['blue', 'green', 'purple', 'orange', 'darkred', 
              'cadetblue', 'darkgreen', 'black', 'darkblue', 'darkpurple']
    
    # Create a feature group for routes for easier toggling
    route_group = folium.FeatureGroup(name="Cable Routes")
    
    # Plot each route
    for i, route in enumerate(routes):
        route_color = colors[i % len(colors)]
        
        # Add route lines
        route_points = [points[node_idx] for node_idx in route]
        folium.PolyLine(
            locations=route_points,
            color=route_color,
            weight=4,
            opacity=0.8,
            popup=f"Route {i+1}"
        ).add_to(route_group)
        
        # Add markers for each point on the route
        for j, node_idx in enumerate(route):
            if node_idx == depot_index:
                continue  # Skip depot, already marked
                
            # Get point info
            point_info = f"Stop {j} on Route {i+1}"
            if points_data is not None:
                point_info += f"<br>{points_data.iloc[node_idx]['source']}"
                
            # Add stop marker
            folium.CircleMarker(
                location=[points[node_idx][0], points[node_idx][1]],
                radius=5,
                color=route_color,
                fill=True,
                fill_color=route_color,
                popup=point_info
            ).add_to(route_group)
    
    # Add the route group to the map
    route_group.add_to(m)
    
    # Add layer control
    folium.LayerControl().add_to(m)
    
    return m

def main():
    # Parse command line arguments
    args = parse_args()
    
    # Load shapefile
    gdf = load_shapefile(args.shapefile)
    
    # Extract points for routing
    points_data = extract_points(gdf)
    
    # If visualize_only flag is set, just create a map of the shapefile and points
    if args.visualize_only:
        print("Visualizing shapefile only without routing...")
        # Create a simple visualization using all points as a single "route"
        points = points_data[0]
        route = list(range(len(points)))
        
        # Create a basic map with just the shapefile and extracted points
        m = folium.Map(location=[points[0][0], points[0][1]], zoom_start=12)
        
        # Add shapefile
        folium.GeoJson(
            gdf,
            name="Shapefile Features",
            style_function=lambda x: {
                'fillColor': '#ffff00',
                'color': '#000000',
                'weight': 1,
                'fillOpacity': 0.1
            }
        ).add_to(m)
        
        # Add points
        points_df = points_data[1]
        for idx, row in points_df.iterrows():
            folium.CircleMarker(
                location=[row['lat'], row['lon']],
                radius=5,
                color='blue',
                fill=True,
                fill_color='blue',
                popup=f"Point {idx}: {row['source']}"
            ).add_to(m)
 
        m.save(args.output)
        print(f"Map with shapefile and points saved to {args.output}")
        return
    
    # Ensure we have enough points
    if len(points_data[0]) < args.num_vehicles + 1:
        print(f"Warning: Not enough points ({len(points_data[0])}) for {args.num_vehicles} vehicles.")
        args.num_vehicles = max(1, len(points_data[0]) - 1)
        print(f"Reducing to {args.num_vehicles} vehicles.")
    
    # Create distance matrix
    distance_matrix = create_distance_matrix(points_data[0])
    
    # Make sure depot_index is valid
    if args.depot_index >= len(points_data[0]):
        print(f"Warning: Depot index {args.depot_index} out of range. Using 0 instead.")
        args.depot_index = 0
    
    # Add capacity constraints for balanced routing
    num_vehicles = min(args.num_vehicles, len(points_data[0]) - 1)
    
    # Solve VRP
    print(f"Solving VRP with {num_vehicles} vehicles and depot at index {args.depot_index}")
    routes = solve_vrp(distance_matrix, num_vehicles, args.depot_index)
    
    if routes:
        # Create map visualization with original shapefile
        m = create_folium_map(points_data, routes, args.depot_index, gdf)
        
        # Save the map
        m.save(args.output)
        print(f"Route map saved to {args.output}")
        
        # Print route summary
        print("\nRoute Summary:")
        for i, route in enumerate(routes):
            total_distance = sum(distance_matrix[route[j]][route[j+1]] 
                                for j in range(len(route)-1))
            stops = [j for j in route if j != args.depot_index]
            print(f"Route {i+1}: {len(stops)} stops, {total_distance/1000:.2f} km")
            
            # Print sequence with source info if available
            sequence = []
            for node in route:
                if points_data[1] is not None:
                    source_info = points_data[1].iloc[node]['source']
                    sequence.append(f"{node} ({source_info})")
                else:
                    sequence.append(str(node))
            print(f"  Sequence: {' -> '.join(sequence)}")
    else:
        print("Could not find a valid solution to the routing problem.")

if __name__ == "__main__":
    main()
