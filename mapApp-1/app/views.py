import requests
from django.http import JsonResponse
from geopy.geocoders import Nominatim
import folium
import pandas as pd
from geopy.distance import geodesic
from rtree import index
import numpy as np
import time

def geocode_address_helper(address):
    if address:
        geolocator = Nominatim(user_agent="geoapi")
        location = geolocator.geocode(address)
        if location:
            return {
                'latitude': location.latitude,
                'longitude': location.longitude,
                'address': location.address
            }
        else:
            return {'error': 'Address not found'}
    else:
        return {'error': 'No address provided'}
    
    
def get_route_helper(start_coords, end_coords):
    try:
        url = f"http://router.project-osrm.org/route/v1/driving/{start_coords[1]},{start_coords[0]};{end_coords[1]},{end_coords[0]}?overview=full&geometries=geojson"
        response = requests.get(url)
        data = response.json()
        
        if data["code"] == "Ok":
            route = data["routes"][0]
            return {
                'geometry': route["geometry"],
                'distance': route["distance"],
                'duration': route["duration"]
            }
        else:
            return {'error': 'Route not found'}
    except Exception as e:
        return {'error': str(e)}


def calculate_route_distances(coordinates):
    route_coords = [(coord[1], coord[0]) for coord in coordinates]
    distances = {} 
    current_distance = 0
    
    # Store the first point with distance 0
    distances[f"{coordinates[0][0]},{coordinates[0][1]}"] = 0
    
    # Calculate cumulative distances along route
    for i in range(1, len(route_coords)):
        prev_point = route_coords[i-1]
        curr_point = route_coords[i]
        segment_distance = geodesic(prev_point, curr_point).miles
        current_distance += segment_distance
        # Store using original lon,lat as key
        distances[f'{coordinates[i][0]},{coordinates[i][1]}'] = current_distance
    
    return distances


def create_map_with_route(route_data):
    
    try:
        if 'error' in route_data:
            return None
            
        coordinates = route_data['geometry']['coordinates']
        # Convert coordinates from [lon, lat] to [lat, lon] as required by folium
        route_coords = [[coord[1], coord[0]] for coord in coordinates]
        
        # Create a map centered on the first point of the route
        m = folium.Map(location=route_coords[0], zoom_start=12)
        
        # Add the route line to the map
        folium.PolyLine(
            route_coords,
            weight=3,
            color='blue',
            opacity=0.8
        ).add_to(m)
        
        # Add markers for start and end points
        folium.Marker(route_coords[0], popup='Start').add_to(m)
        folium.Marker(route_coords[-1], popup='End').add_to(m)
        
        return m
    except Exception as e:
        return None
    

def distance_to_segment(Q, P0, P1):
    x0, y0 = P0
    x1, y1 = P1
    x, y = Q
    dx, dy = x1 - x0, y1 - y0
    if dx == dy == 0:
        return geodesic(P0, Q).miles

    t = ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))

    closest_point = (x0 + t * dx, y0 + t * dy)

    return geodesic(Q, closest_point).miles

def create_rtree(polyline):
    # Create an R-tree index
    idx = index.Index()
    for i, (P0, P1) in enumerate(zip(polyline[:-1], polyline[1:])):
        # Get the bounding box of the segment
        x_min, y_min = min(P0[0], P1[0]), min(P0[1], P1[1])
        x_max, y_max = max(P0[0], P1[0]), max(P0[1], P1[1])
        # Insert the bounding box into the R-tree with the segment index
        idx.insert(i, (x_min, y_min, x_max, y_max))
    return idx

def distance_to_polyline_with_rtree(Q, polyline, idx):
    min_distance = float('inf')
    # Query the R-tree for candidate segments near Q
    x, y = Q
    candidates = idx.nearest((x, y, x, y), num_results=5)  # Adjust num_results as needed
    for i in candidates:
        P0, P1 = polyline[i], polyline[i + 1]
        dist = distance_to_segment(Q, P0, P1)
        if dist < min_distance:
            min_distance = dist
            lat, lon = P0[0], P0[1]
    return min_distance, lon, lat

def get_gas_stations_on_route(route_coords, csv_file_path, distances, max_distance_m=1):
    
    
    stations_df = pd.read_csv(csv_file_path)
    
    # Convert coordinates from [lon, lat] to [lat, lon] as required by geopy
    route_coords = np.array([[coord[1], coord[0]] for coord in route_coords])
    
    
    # Create an R-tree index for the route
    idx = create_rtree(route_coords)
    
    # Find gas stations near the route
    gas_stations = []
    for _, station in stations_df.iterrows():
        station_coords = (station['latitude'], station['longitude'])
        distance_to_route, cur_lat, cur_lon = distance_to_polyline_with_rtree(station_coords, route_coords, idx)
        if distance_to_route <= max_distance_m:
            gas_stations.append({
                'name': station['Truckstop Name'],
                'latitude': cur_lat,
                'longitude': cur_lon,
                'distance_to_route': distance_to_route,
                'price': station['Retail Price'],
                'distance_from_start': round(distances[f'{cur_lat},{cur_lon}'])
            })

    return gas_stations


def optimum_gas_stations(n, stations):
    MAX_RANGE = 500
    dp = [[float('inf')] * (MAX_RANGE + 5) for _ in range(n + 10)]
    vis = [[0] * (MAX_RANGE + 5) for _ in range(n + 10)]
    optimal_stations = []

    def sol(i, dis):
        if i == n - 1:
            return 0.0
        if vis[i][dis] == 1:
            return dp[i][dis]
        
        vis[i][dis] = 1
        ret = float('inf')
        
        # Try without refueling at current station
        if (stations[i + 1]["distance_from_start"] - stations[i]["distance_from_start"]) + dis <= MAX_RANGE:
            ret = sol(i + 1, dis + (stations[i + 1]["distance_from_start"] - stations[i]["distance_from_start"]))
            
        # Try with refueling at current station
        if (stations[i + 1]["distance_from_start"] - stations[i]["distance_from_start"]) <= MAX_RANGE:
            cost = sol(i + 1, stations[i + 1]["distance_from_start"] - stations[i]["distance_from_start"]) + stations[i]["price"] * (dis / 10.0)
            ret = min(ret, cost)
            
        dp[i][dis] = ret
        return ret

    def build(i, dis):
        if i == n - 1:
            return
        
        # Try without refueling
        if (stations[i + 1]["distance_from_start"] - stations[i]["distance_from_start"]) + dis <= MAX_RANGE:
            no_refuel_cost = sol(i + 1, dis + (stations[i + 1]["distance_from_start"] - stations[i]["distance_from_start"]))
            if abs(dp[i][dis] - no_refuel_cost) < 1e-9:
                build(i + 1, dis + (stations[i + 1]["distance_from_start"] - stations[i]["distance_from_start"]))
                return
        
        # Try with refueling
        if (stations[i + 1]["distance_from_start"] - stations[i]["distance_from_start"]) <= MAX_RANGE:
            refuel_cost = sol(i + 1, stations[i + 1]["distance_from_start"] - stations[i]["distance_from_start"]) + stations[i]["price"] * (dis / 10.0)
            if abs(dp[i][dis] - refuel_cost) < 1e-9:
                optimal_stations.append(stations[i])
                build(i + 1, stations[i + 1]["distance_from_start"] - stations[i]["distance_from_start"])
                return

    minimum_cost = sol(0, 0)
    build(0, 0)
    return optimal_stations, minimum_cost

    
    
def route_api(request):
    if request.method == 'GET':
        
        start = request.GET.get('start')
        destination = request.GET.get('destination')

        if not start or not destination:
            return JsonResponse({'error': 'Start and destination addresses are required'})

        # Geocode both addresses
        start_location = geocode_address_helper(start)
        time.sleep(1)
        end_location = geocode_address_helper(destination)

        if 'error' in start_location or 'error' in end_location:
            return JsonResponse({'error': 'Unable to geocode one or both addresses'})

        # Get the route
        start_coords = (start_location['latitude'], start_location['longitude'])
        end_coords = (end_location['latitude'], end_location['longitude'])
        route_data = get_route_helper(start_coords, end_coords)

        if 'error' in route_data:
            return JsonResponse({'error': 'Unable to find route'})

        distances = calculate_route_distances(route_data['geometry']['coordinates'])

        # Create the map
        map_obj = create_map_with_route(route_data)
        
        if map_obj is None:
            return JsonResponse({'error': 'Unable to create map'})

        
        
        # Get gas stations along the route
        csv_file_path = 'app/data/final.csv'
        gas_stations = get_gas_stations_on_route(route_data['geometry']['coordinates'], csv_file_path, distances)
        
        gas_stations.append({
            'name': 'Start',
            'latitude': start_location['latitude'],
            'longitude': start_location['longitude'],
            'distance_to_route': 0,
            'price': 0,
            'distance_from_start': 0
        })
        gas_stations.append({
            'name': 'Destination',
            'latitude': end_location['latitude'],
            'longitude': end_location['longitude'],
            'distance_to_route': 0,
            'price': 0,
            'distance_from_start': round(distances[f'{route_data["geometry"]["coordinates"][-1][0]},{route_data["geometry"]["coordinates"][-1][1]}'])
        })
        gas_stations = sorted(gas_stations, key=lambda x: x['distance_from_start'])
        
        # Get the optimal gas stations
        n = len(gas_stations)
        optimal_stations,min_money = optimum_gas_stations(n, gas_stations)
        
        for station in optimal_stations:
            folium.Marker([station['longitude'], station['latitude']], popup=station['name']).add_to(map_obj)
        map_obj.save("map.html")
        map_html = map_obj._repr_html_()
        
        
        
        response_data = {
            'route': route_data,
            'map_html': map_html,
            'gas_stations': optimal_stations,
            'start_location': start_location,
            'end_location': end_location,
            'min_money': min_money
        }

        return JsonResponse(response_data)

    return JsonResponse({'error': 'Only GET requests are supported'})