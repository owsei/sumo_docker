import sumolib
# import traci
import os
import sys
import subprocess
import json
import tempfile
import requests
import time
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional, List, Dict
import uuid
# from constants import PREFIX, DOUBLE_ROWS, ROW_DIST, SLOTS_PER_ROW, SLOT_WIDTH
sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
from sumolib import checkBinary
import traci
from urllib.parse import unquote
import platform

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

class SimulationParams(BaseModel):
    num_vehicles: int = 50
    duration_sec: int = 300
    # Lista de IDs de "edges" (vías) prohibidas
    blocked_edges: List[str] = []

class  MensajeSocket(BaseModel):
    mensaje: str
    color: str
    tipo: str

def download_osm_data(bbox: BoundingBox, output_path: str):
    """Descarga directa de Overpass API para evitar errores de osmGet.py"""
    print("Descargando datos de OSM")
    types_filter = "|".join(bbox.road_types)
    print("Filtros de tipos: ", types_filter)
    # Overpass usa el orden: south, west, north, east
    overpass_url = "https://maps.mail.ru/osm/tools/overpass/api/interpreter"
    # Esta query descarga solo las vías (ways) que coincidan con los tipos
    # y también los nodos (nodes) que forman esas vías.
    query = f"""
    [out:xml][timeout:25];
    (
      way["highway"~"{types_filter}"]["access"!="private"]["motor_vehicle"!="no"]({bbox.south},{bbox.west},{bbox.north},{bbox.east});
      (._;>;);
    );
    out meta;
    """
    print("Query de OSM: ", query)
    response = requests.get(overpass_url, params={'data': query})
    if response.status_code == 200:
        with open(output_path, "wb") as f:
            f.write(response.content)
    else:
        print("Error al conectar con Overpass: ", response.status_code)
        raise Exception(f"Error al conectar con Overpass: {response.status_code}")

def generate_net(sumo_home ,bbox):
    
    osm_file = os.path.join(tmpdir, "mapa.osm.xml")
    net_file = os.path.join(tmpdir, "mapa.net.xml")
    route_file = os.path.join(tmpdir, "mapa.rou.xml")
    
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

def generate_traffic(sumo_home, net_file, route_file):
    random_trips = os.path.join(sumo_home, "tools", "randomTrips.py")
    subprocess.run([
        "python", random_trips,
        "-n", net_file,
        "-r", route_file,
        "-e", "3600",  # Simular 3600 segundos de tráfico
        "--period", "10" # Aparece un coche cada 0.5 segundos
    ], check=True)
    
def wait_for_file(filepath, timeout=10):
    """
    Bloquea la ejecución hasta que el archivo existe y es accesible.
    """
    start_time = time.time()
    while time.time() - start_time < timeout:
        if os.path.exists(filepath):
            try:
                # Intentamos abrirlo en modo lectura para verificar que no está bloqueado
                with open(filepath, 'r') as f:
                    if len(f.read(1)) > 0: # Verificamos que no esté vacío
                        return True
            except IOError:
                # El archivo existe pero está siendo usado por otro proceso
                pass
        time.sleep(0.1) # Espera de 100ms antes de reintentar
    return False

def convert_net_to_geojson_net(net_file):
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

def convert_to_geojson_traci(simulation_data: List[Dict]) -> Dict:
    """Convierte los datos de simulación a GeoJSON para Cesium"""
    features = []
    
    for step_data in simulation_data:
        for vehicle in step_data["vehicles"]:
            feature = {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [vehicle["longitude"], vehicle["latitude"]]
                },
                "properties": {
                    "id": vehicle["id"],
                    "speed": vehicle["speed"],
                    "angle": vehicle["angle"],
                    "time": vehicle["time"]
                }
            }
            features.append(feature)
    
    return {
        "type": "FeatureCollection",
        "features": features
    }

async def convert_to_websocket(simulation_data: List[Dict]) -> Dict:
    """Convierte los datos de simulación a GeoJSON para Cesium"""
    features = []
    
    for step_data in simulation_data:
        for vehicle in step_data["vehicles"]:
            feature = {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [vehicle["longitude"], vehicle["latitude"]]
                },
                "properties": {
                    "id": vehicle["id"],
                    "speed": vehicle["speed"],
                    "angle": vehicle["angle"],
                    "time": vehicle["time"]
                }
            }
            await websocket.send_json({feature})  
    
async def sendCreateRoads(net_file):
    net = sumolib.net.readNet(net_file)
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
            "carriles": edge.getLaneNumber(),
            
        }

        feature = {
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": coords
            },
            "properties": properties
        }
        await websocket.send_json(feature)
    

@app.get("/")
async def root():
    return {"status": "ok"}

@app.post("/get-roads")
async def get_roads(bbox: BoundingBox):
    # ... (código anterior para descargar OSM y ejecutar netconvert) ...
    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        # Intenta buscar la ruta por defecto si no está la variable
        sumo_home = r"C:\Program Files (x86)\Eclipse\Sumo"
    print("Ruta de SUMO encontrada correctamente", sumo_home)
    with tempfile.TemporaryDirectory() as tmpdir:
        osm_file = os.path.join(tmpdir, "mapa.osm.xml")
        net_file = os.path.join(tmpdir, "mapa.net.xml")

        try:
            # 1. Descarga (usando el método de requests que vimos antes)
            print("Descargando datos de OSM")
            download_osm_data(bbox, osm_file)
            print("Datos de OSM descargados correctamente")
            sumo_types = ",".join([f"highway.{t}" for t in bbox.road_types])
            print("Tipos de carreteras seleccionados correctamente")
            # 2. Generar red de SUMO
            print("Generando red de SUMO")
            subprocess.run([
                os.path.join(sumo_home, "bin", "netconvert"),
                "--osm-files", osm_file,
                "--output-file", net_file,
                "--geometry.remove", "true",
                "--proj.utm", "true",
                "--keep-edges.by-type", sumo_types, # <--- Mantiene solo estos tipos
                "--remove-edges.isolated", "true"
            ], check=True)  
            print("Red de SUMO generada correctamente")
            # 3. EN LUGAR DE LLAMAR A net2geojson.py, USAMOS NUESTRA FUNCIÓN
            geojson_data = convert_net_to_geojson_net(net_file)
            print("GeoJSON generado correctamente")
            return geojson_data

        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

#**************************WEBSOCKET******************************#

@app.websocket("/ws/simulation")
async def websocket_simulation(websocket: WebSocket):
    await websocket.accept()
    bbox_str = websocket.query_params.get("bbox")
    forbiddenRoads = websocket.query_params.get("forbiddenRoads")
    num_vehicles = int(websocket.query_params.get("num_vehicles"))
    print("Numero de vehiculos: ", num_vehicles)
    duration_sec = int(websocket.query_params.get("duration_sec"))
    

    forbiddenRoads = unquote(forbiddenRoads)
    forbiddenRoadsArray = json.loads(forbiddenRoads)

    if not bbox_str:
        raise HTTPException(status_code=400, detail="Missing bbox parameter")
    
    try:
        bbox = BoundingBox(**json.loads(bbox_str))
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid bbox format")
    
   
    #DETERMINA EL SISTEMA OPERATIVO SOBRE EL QUE SE EJECUTA LA APLICACION
    operativeSytemIsLinux= 0 if platform.system()=="Linux" else 1
    if operativeSytemIsLinux==0:
       sumo_home = "/usr/share/sumo"
    else:
        sumo_home = r"C:\Program Files (x86)\Eclipse\Sumo"

    print("Ruta de SUMO encontrada correctamente", sumo_home,"Operative system",platform.system())
    await websocket.send_json({"mensaje":"Ruta de SUMO encontrada correctamente"+ sumo_home +"|Operative system"+platform.system()})

    with tempfile.TemporaryDirectory() as tmpdir:
        print("Directorio temporal creado correctamente", tmpdir)
        osm_file = os.path.join(tmpdir, "mapa.osm.xml")
        print("Archivo OSM creado correctamente", osm_file)
        net_file = os.path.join(tmpdir, "mapa.net.xml")
        print("Archivo NET creado correctamente", net_file)
        route_file = os.path.join(tmpdir, "mapa.rou.xml")
        print("Archivo ROUT creado correctamente", route_file)

        # 1. Descarga (usando el método de requests que vimos antes)
        print("Descargando datos de OSM")
        await websocket.send_json({"mensaje":"Descargando datos OSM"})

        download_osm_data(bbox, osm_file)
        print("Datos de OSM descargados correctamente")
        await websocket.send_json({"mensaje":"Datos de OSM descargados correctamente"})

        sumo_types = ",".join([f"highway.{t}" for t in bbox.road_types])
        
        # 2. Generar red de SUMO
        print("Generando red de SUMO")
        await websocket.send_json({"mensaje":"Generando red de SUMO"})

        
        
        if operativeSytemIsLinux==0:
            subprocess.run([
                "netconvert",
                "--osm-files", osm_file,
                "--output-file", net_file,
                "--geometry.remove", "true",
                "--proj.utm", "true",
                "--keep-edges.by-type", sumo_types, # <--- Mantiene solo estos tipos
                "--remove-edges.isolated", "true"
        ], check=True)
        else:
            subprocess.run([
                os.path.join(sumo_home, "bin", "netconvert"),
                "--osm-files", osm_file,
                "--output-file", net_file,
            "--geometry.remove", "true",
            "--proj.utm", "true",
            "--keep-edges.by-type", sumo_types, # <--- Mantiene solo estos tipos
            "--remove-edges.isolated", "true"
        ], check=True)



        print("Red generada correctamente")
        await websocket.send_json({"mensaje":"Red generada correctamente"})

        # 3. Generar tráfico aleatorio
        print("Generando tráfico aleatorio")
        await websocket.send_json({"mensaje":"Generando tráfico aleatorio"})
        

        period = duration_sec / num_vehicles if num_vehicles > 0 else 100
        print("Periodo: ", period)
        await websocket.send_json({"mensaje":"Periodo de aparicion de vehiculos: "+ str(period)})
    
        random_trips = os.path.join(sumo_home, "tools", "randomTrips.py")
        if operativeSytemIsLinux==0:
            procesoTraffic=subprocess.run([
                "python3", random_trips,
                "-n", net_file,
                "-r", route_file,
                "-e", str(duration_sec),  # Simular 3600 segundos de tráfico
            "--period", str(period), # Aparece un coche cada 0.5 segundos
            "--fringe-factor", "1000"
            
            ], check=True)  
        else:
            procesoTraffic=subprocess.run([
                "python", random_trips,
                "-n", net_file,
                "-r", route_file,
                "-e", str(duration_sec),  # Simular 3600 segundos de tráfico
            "--period", str(period), # Aparece un coche cada 0.5 segundos
            "--fringe-factor", "1000"
            
            ], check=True)  
        print("Tráfico generado correctamente para "+ str(num_vehicles) + " vehiculos") 
        await websocket.send_json({"mensaje":"Tráfico generado correctamente para "+ str(num_vehicles) + " vehiculos"})

        # Primero crea el archivo de configuración SUMO
        print("Creando archivo de configuración SUMO")
        await websocket.send_json({"mensaje":"Creando archivo de configuración SUMO"})
        
        config_file = os.path.join(tmpdir, "simulation.sumocfg")
        print("Archivo de configuración SUMO creado correctamente", config_file)
        await websocket.send_json({"mensaje":"Archivo de configuración SUMO creado correctamente"})
        
        # Crear el archivo .sumocfg
        with open(config_file, 'w') as f:
            f.write(f"""<?xml version="1.0" encoding="UTF-8"?>
                <configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.xsd">
                    <input>
                        <net-file value="{net_file}"/>
                        <route-files value="{route_file}"/>
                    </input>
                    <time>
                        <begin value="0"/>
                        <end value="{str(duration_sec)}"/>
                    </time>
                </configuration>""")

        print("Archivos de configuración SUMO generados correctamente")

        # 4. Iniciar simulación con TraCI
        try:
             
            traciBinary = os.path.join(sumo_home, "bin", "sumo")  # sin GUI
            print("Iniciando simulación")
            await websocket.send_json({"mensaje":"Iniciando simulación"})
            traci.start([
                traciBinary,
                "-c", config_file,
                "--step-length", "0.1",  # 1 segundo por paso
                "--no-warnings", "true",
                "--device.rerouting.probability", "1.0", # Todos los coches pueden recalcular
                "--device.rerouting.period", "1",        # Recalcular en cuanto cambie algo
                "--device.rerouting.pre-period", "0",
                "--ignore-route-errors", "true"          # <--- ESTO EVITA QUE LA SIMULACIÓN SE PARE
            ])
            
            edges=traci.edge.getIDList()
            print("Total de calles: ", len(edges))
            await websocket.send_json({"mensaje":"Total de calles: "+ str(len(edges))})

            for edge_id in edges:
                if (edge_id in forbiddenRoadsArray):
                    traci.edge.setAllowed(edge_id, [])
                    traci.edge.setEffort(edge_id, 999999)
                    print("Calle prohibida: ", edge_id)
                    await websocket.send_json({"mensaje":"Calle prohibida: "+ edge_id})
            
            # Ejecutar la simulación paso a paso
            step = 0
            max_steps = duration_sec  # Duracion de la simulación
            
            while step < max_steps and traci.simulation.getMinExpectedNumber() > 0:
                try:
                    traci.simulationStep()  # Avanzar un paso
                    
                    vehicles_at_step = []
                    for veh in traci.vehicle.getIDList():
                        traci.vehicle.rerouteTraveltime(veh)
                        # Obtener posición (x, y) en la proyección de SUMO
                        x, y = traci.vehicle.getPosition(veh)
                        
                        # Convertir a lon/lat (SUMO usa coordenadas proyectadas)
                        lon, lat = traci.simulation.convertGeo(x, y)
                        
                        # Obtener otros datos útiles
                        speed = traci.vehicle.getSpeed(veh)
                        angle = traci.vehicle.getAngle(veh)
                        
                        vehiculo={
                            "id": veh,
                            "longitude": lon,
                            "latitude": lat,
                            "speed": speed,
                            "angle": angle,
                            "time": step
                        }

                        await websocket.send_json(vehiculo)
                    
                    step += 1
                except traci.TraCIException as e:
                    if "has no valid route" in str(e):
                        print("Detectado error de ruta, saltando vehículo conflictivo...")
                        # El parámetro --ignore-route-errors en el start suele bastar,
                        # pero aquí podrías manejar lógica extra.
                
            # Cerrar TraCI
            traci.close()
            print("Simulación finalizada correctamente")
            await websocket.send_json({"mensaje":"Simulación finalizada correctamente"})
            await websocket.close()
            
                
            
            
        except Exception as e:
            print(f"Error en la simulación:")
            await websocket.send_json({"mensaje":"Error en la simulación: "+str(e)})
            if traci.isLoaded():
                traci.close()
            raise HTTPException(status_code=500, detail=str(e))

# WEBSOCKET PARA PROBAR LA CONEXIÓN
@app.websocket("/ws/test")
async def test_websocket(websocket: WebSocket):
    await websocket.accept()
    mensaje = MensajeSocket(
        mensaje="Conexión exitosa",
        color="#00FF00",
        tipo="success"
    )
    await websocket.send_json(mensaje.dict())
    await websocket.close()

# WEBSOCKET PARA OBTENER LAS CARRETERAS
@app.websocket("/ws/getRoads")
async def get_roads_websocket(websocket: WebSocket):
    await websocket.accept()
    bbox_str = websocket.query_params.get("bbox")
    if not bbox_str:
        await websocket.send_json({"error": "No se proporcionó bbox"})
        return
    try:
        bbox = BoundingBox(**json.loads(bbox_str))
    except json.JSONDecodeError:
        await websocket.send_json({"error": "bbox inválido"})
        return
    
    await websocket.send_json({"mensaje": "bbox recibido correctamente"})
    
    sumo_home = os.environ.get("SUMO_HOME")

    if not sumo_home:
        sumo_home = r"C:\Program Files (x86)\Eclipse\Sumo"

    print("Ruta de SUMO encontrada correctamente", sumo_home)
    with tempfile.TemporaryDirectory() as tmpdir:
        osm_file = os.path.join(tmpdir, "mapa.osm.xml")
        net_file = os.path.join(tmpdir, "mapa.net.xml")
        net_file = os.path.join(tmpdir, "pamplona_v1.net.xml")

        try:
            # 1. Descarga (usando el método de requests que vimos antes)
            print("Descargando datos de OSM")
            download_osm_data(bbox, osm_file)
            print("Datos de OSM descargados correctamente")
            sumo_types = ",".join([f"highway.{t}" for t in bbox.road_types])
            print("Tipos de carreteras seleccionados correctamente")
            # 2. Generar red de SUMO
            print("Generando red de SUMO")
            subprocess.run([
                os.path.join(sumo_home, "bin", "netconvert"),
                "--osm-files", osm_file,
                "--output-file", net_file,
                "--geometry.remove", "true",
                "--proj.utm", "true",
                "--keep-edges.by-type", sumo_types, # <--- Mantiene solo estos tipos
                "--remove-edges.isolated", "true"
            ], check=True)  
            print("Red de SUMO generada correctamente")
            # 3. EN LUGAR DE LLAMAR A net2geojson.py, USAMOS NUESTRA FUNCIÓN
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
                await websocket.send_json(feature)
            await websocket.send_json({"mensaje": "Descarga de carreteras finalizada correctamente👍"})
            await websocket.close()

        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

        
@app.websocket("/ws/getRoadsPamplona")
async def get_roads_websocket(websocket: WebSocket):
    await websocket.accept()

    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        sumo_home = r"C:\Program Files (x86)\Eclipse\Sumo"

    print("Ruta de SUMO encontrada correctamente", sumo_home)
    with tempfile.TemporaryDirectory() as tmpdir:
        net_file = os.path.join(tmpdir, "pamplona_v1.net.xml")

        try:
            # 1. Descarga (usando el método de requests que vimos antes)
            print("Red de SUMO generada correctamente")
            # 3. EN LUGAR DE LLAMAR A net2geojson.py, USAMOS NUESTRA FUNCIÓN
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
                await websocket.send_json(feature)
            await websocket.send_json({"mensaje": "Descarga de carreteras finalizada correctamente👍"})
            await websocket.close()

        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))