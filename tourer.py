import osmnx as ox
import folium
from geopy.geocoders import Nominatim
import random

# Funktion zum Abrufen des aktuellen Standorts
def get_location():
    geolocator = Nominatim(user_agent="route_planner")
    location = geolocator.geocode("Eichkoegl, Österreich")  # Testadresse für Standort
    if location:
        return location.latitude, location.longitude
    else:
        raise ValueError("Standort konnte nicht gefunden werden.")

# Funktion zum Berechnen einer Route auf existierenden Straßen
def generate_route(lat, lon, distance_km, mode='walk'):
    # Laden des Straßennetzes in der Nähe des Startpunkts
    G = ox.graph_from_point((lat, lon), dist=distance_km*1000, network_type=mode)

    # Nächster Punkt im Netzwerk
    start_node = ox.nearest_nodes(G, lon, lat)

    # Wähle einen zufälligen Endpunkt im Netzwerk aus einer Distanz von bis zu 75% der gewünschten Strecke
    nodes = list(G.nodes)
    end_node = random.choice(nodes)

    # Generiere eine Route zwischen den beiden Punkten
    route = ox.distance.shortest_path(G, start_node, end_node, weight='length', cpus=1)

    # Route in Koordinaten umwandeln
    route_coords = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in route]
    
    return route_coords

# Funktion zum Anzeigen der Route auf einer Karte
def show_route(route_points):
    start_lat, start_lon = route_points[0]
    
    # Karte erstellen
    map_route = folium.Map(location=[start_lat, start_lon], zoom_start=13)

    # Route hinzufügen
    folium.PolyLine(route_points, color="blue", weight=2.5, opacity=1).add_to(map_route)

    # Karte speichern und anzeigen
    map_route.save("route.html")
    print("Route gespeichert in route.html")

# Funktion zur Validierung der Distanz-Eingabe
def get_distance():
    while True:
        try:
            distance_km = float(input("Wie viele Kilometer möchtest du laufen/fahren? "))
            return distance_km
        except ValueError:
            print("Bitte gib eine gültige Zahl ein.")

if __name__ == "__main__":
    # Schritt 1: Aktuellen Standort abrufen
    lat, lon = get_location()
    print(f"Aktueller Standort: Lat {lat}, Lon {lon}")

    # Schritt 2: Benutzereingabe für Distanz
    distance_km = get_distance()

    # Schritt 3: Route basierend auf existierenden Straßen generieren
    route_points = generate_route(lat, lon, distance_km)

    # Schritt 4: Route anzeigen
    if route_points:
        show_route(route_points)
