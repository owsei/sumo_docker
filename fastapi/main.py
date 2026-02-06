import sumolib
import traci
import os
import subprocess
import json
import tempfile
import requests
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional, List

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class BoundingBox(BaseModel):
    west: float
    south: float
    east: float
    north: float
    road_types: Optional[List[str]] = ["motorway", "primary", "secondary", "tertiary", "residential"]

def download_osm_data(bbox: BoundingBox, output_path: str):
    """Descarga directa de Overpass API para evitar errores de osmGet.py"""
    types_filter = "|".join(bbox.road_types)
    # Overpass usa el orden: south, west, north, east
    overpass_url = "https://overpass-api.de/api/interpreter"
    # Esta query descarga solo las vías (ways) que coincidan con los tipos
    # y también los nodos (nodes) que forman esas vías.
    query = f"""
    [out:xml][timeout:25];
    (
      way["highway"~"{types_filter}"]({bbox.south},{bbox.west},{bbox.north},{bbox.east});
      (._;>;);
    );
    out meta;
    """
    response = requests.get(overpass_url, params={'data': query})
    if response.status_code == 200:
        with open(output_path, "wb") as f:
            f.write(response.content)
    else:
        raise Exception(f"Error al conectar con Overpass: {response.status_code}")

def generate_traffic(sumo_home, net_file, route_file):
    random_trips = os.path.join(sumo_home, "tools", "randomTrips.py")
    subprocess.run([
        "python", random_trips,
        "-n", net_file,
        "-r", route_file,
        "-e", "3600",  # Simular 3600 segundos de tráfico
        "--period", "0.5" # Aparece un coche cada 0.5 segundos
    ], check=True)

@app.get("/")
async def root():
    return {"status": "ok"}

# @app.post("/get-roads")
# async def get_roads(bbox: BoundingBox):
#     sumo_home = os.environ.get("SUMO_HOME")
#     if not sumo_home:
#         # Intenta buscar la ruta por defecto si no está la variable
#         sumo_home = r"C:\Program Files (x86)\Eclipse\Sumo"
        
#     # Rutas de herramientas
#     netconvert = os.path.join(sumo_home, "bin", "netconvert")
#     net2geojson = os.path.join(sumo_home, "tools", "net", "net2geojson.py")

#     with tempfile.TemporaryDirectory() as tmpdir:
#         osm_file = os.path.join(tmpdir, "mapa.osm.xml")
#         net_file = os.path.join(tmpdir, "mapa.net.xml")
#         geojson_file = os.path.join(tmpdir, "mapa.geojson")

#         try:
#             # 1. Descarga directa (Adiós al error de osmGet.py)
#             download_osm_data(bbox, osm_file)

#             # 2. Convertir a red de SUMO
#             # Añadimos --proj.utm para que convierta bien las coordenadas de Pamplona
#             subprocess.run([
#                 netconvert, 
#                 "--osm-files", osm_file, 
#                 "--output-file", net_file,
#                 "--geometry.remove", "true",
#                 "--roundabouts.guess", "true",
#                 "--proj.utm", "true" 
#             ], check=True)

#             # 3. Convertir a GeoJSON para Cesium
#             subprocess.run([
#                 "python", net2geojson, 
#                 "-n", net_file, 
#                 "-o", geojson_file,
#                 "--lanes",
#                 "--internal"
#             ], check=True)

#             with open(geojson_file, "r") as f:
#                 data = json.load(f)
            
#             return data

#         except Exception as e:
#             print(f"Error detallado: {str(e)}")
#             raise HTTPException(status_code=500, detail=str(e))

def convert_net_to_geojson_custom(net_file):
    """
    Usa sumolib para leer la red de SUMO y crear un GeoJSON 
    con nombre de calle, tipo y otros atributos.
    """
    # Cargamos la red
    net = sumolib.net.readNet(net_file)
    features = []

    for edge in net.getEdges():
        # Obtenemos la geometría (forma) de la carretera
        # Convertimos las coordenadas internas de SUMO a Lon/Lat
        shape = edge.getShape()
        coords = [net.convertXY2LonLat(x, y) for x, y in shape]

        # Extraemos las propiedades que queremos
        # Nota: edge.getName() devuelve el nombre de la calle de OSM
        properties = {
            "id": edge.getID(),
            "nombre": edge.getName() or "Calle sin nombre",
            "tipo": edge.getType(),
            "velocidad_max": edge.getSpeed() * 3.6, # Convertir m/s a km/h
            "carriles": edge.getLaneNumber()
        }

        feature = {
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": coords
            },
            "properties": properties
        }
        features.append(feature)

    return {
        "type": "FeatureCollection",
        "features": features
    }


@app.post("/get-roads")
async def get_roads(bbox: BoundingBox):
    # ... (código anterior para descargar OSM y ejecutar netconvert) ...
    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        # Intenta buscar la ruta por defecto si no está la variable
        sumo_home = r"C:\Program Files (x86)\Eclipse\Sumo"

    with tempfile.TemporaryDirectory() as tmpdir:
        osm_file = os.path.join(tmpdir, "mapa.osm.xml")
        net_file = os.path.join(tmpdir, "mapa.net.xml")

        try:
            # 1. Descarga (usando el método de requests que vimos antes)
            download_osm_data(bbox, osm_file)
            sumo_types = ",".join([f"highway.{t}" for t in bbox.road_types])
            # 2. Generar red de SUMO
            subprocess.run([
                os.path.join(sumo_home, "bin", "netconvert"),
                "--osm-files", osm_file,
                "--output-file", net_file,
                "--geometry.remove", "true",
                "--proj.utm", "true",
                "--keep-edges.by-type", sumo_types, # <--- Mantiene solo estos tipos
                "--remove-edges.isolated", "true"
            ], check=True)

            # 3. EN LUGAR DE LLAMAR A net2geojson.py, USAMOS NUESTRA FUNCIÓN
            geojson_data = convert_net_to_geojson_custom(net_file)

            return geojson_data

        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))


@app.post("/simulate")
async def start_simulation(bbox: BoundingBox):
    # ... (pasos anteriores para tener el net_file) ...

    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        # Intenta buscar la ruta por defecto si no está la variable
        sumo_home = r"C:\Program Files (x86)\Eclipse\Sumo"

    with tempfile.TemporaryDirectory() as tmpdir:
        route_file = os.path.join(tmpdir, "mapa.rou.xml")
        net_file = os.path.join(tmpdir, "mapa.net.xml")
        generate_traffic(sumo_home, net_file, route_file)

        # Iniciar simulación en segundo plano (sin interfaz gráfica)
        traci.start(["sumo", "-n", net_file, "-r", route_file])
        
        simulation_data = [] # Aquí guardaremos los pasos

        for step in range(3600): # Simular 100 pasos
            traci.simulationStep()
            vehicles = traci.vehicle.getIDList()
            
            step_frame = []
            for veh_id in vehicles:
                # Convertir coordenadas de SUMO (x,y) a Lon/Lat para Cesium
                x, y = traci.vehicle.getPosition(veh_id)
                lon, lat = net.convertXY2LonLat(x, y)
                
                step_frame.append({
                    "id": veh_id,
                    "lon": lon,
                    "lat": lat,
                    "angle": traci.vehicle.getAngle(veh_id) # Útil para orientar el coche en 3D
                })
            
            simulation_data.append({"step": step, "vehicles": step_frame})
        
        traci.close()
        return simulation_data


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)