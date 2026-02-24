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
import asyncio
from pyproj import Geod
import math

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
            "nombre": edge.getStreetName() or "Calle sin nombre",
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

    # Obtener todos los sistemas de semáforos (TLS)
    semaforos = net.getTrafficLights()

    for tls in semaforos:
        tls_id = tls.getID()
        # Los semáforos están asociados a nodos (junctions)
        nodos = tls.getNodes()
        for nodo in nodos:
            x, y = nodo.getCoord()
            lon, lat = net.convertXY2LonLat(x, y)
            properties = {
                "id": tls_id,
                "tipo": "trafficlight"
            }
            feature = {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [lon, lat]
                },
                "properties": properties
            }
            features.append(feature)

    return {
        "type": "FeatureCollection",
        "features": features
    }



def getVelocityStyle(velocity):

    if (velocity>119):
        return "blue"
    elif (velocity>101 and velocity<=119):
        return "green"
    elif (velocity>80 and velocity<=101):
        return "yellow"
    elif (velocity>60 and velocity<=80):
        return "purple"
    elif (velocity>40 and velocity<=60):
        return "orange"
    elif (velocity>20 and velocity<=40):
        return "white"
    else:
        return "gray"

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

def calcular_heading_preciso(lat1, lon1, lat2, lon2):
    geod = Geod(ellps="WGS84")
    # fwd_azimuth es el ángulo hacia adelante (azimut)
    # inv_azimuth es el ángulo de regreso
    fwd_azimuth, inv_azimuth, distance = geod.inv(lon1, lat1, lon2, lat2)
    
    # pyproj devuelve grados de -180 a 180. 
    # Cesium prefiere radianes.
    heading_rad = math.radians((fwd_azimuth + 360) % 360)
    return heading_rad 

def getTrafficLightColor(state):
    match state:
        case "r" | "R":
            return "red"
        case "y" | "Y":
            return "yellow"
        case "g" | "G":
            return "green"
        case _:
            return "gray"  

@app.get("/")
async def root():
    return {"status": "ok"}

@app.get("/status")
async def status():
    return {"status": "ok"}


@app.websocket("/ws/status")
async def websocket_status(websocket: WebSocket):
    try:
        await websocket.accept()
        await websocket.send_json(estado_simulador)

    except Exception as e:
        print(f"Error en WebSocket: {e}")
    finally:
        await websocket.close()

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
                "--remove-edges.isolated", "true",
                "--tls.guess","true",
                "--tls.join", "true"
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
    print("Duracion de la simulacion: ", duration_sec)
    zonaSnachoFuerte = int(websocket.query_params.get("zonaSnachoFuerte"))
    print("Zona Snacho Fuerte: ", zonaSnachoFuerte)
    

    forbiddenRoads = unquote(forbiddenRoads)
    forbiddenRoadsArray = json.loads(forbiddenRoads)

    if zonaSnachoFuerte==0: 
        if not bbox_str:
            raise HTTPException(status_code=400, detail="Missing bbox parameter")
        else:
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

        # tipos_vehiculos = os.path.join(tmpdir, "tipos_vehiculos.add.xml")
        # print("Archivo tipos_vehiculos creado correctamente", tipos_vehiculos)

        detalles_viajes = os.path.join(tmpdir, "detalles_viajes.xml")
        print("Archivo detalles_viajes creado correctamente", detalles_viajes)

        informe_final = os.path.join(tmpdir, "informe_final.xml")
        print("Archivo informe_final creado correctamente", informe_final)
        
        emisiones_por_calle = os.path.join(tmpdir, "emisiones_por_calle.xml")
        print("Archivo emisiones_por_calle creado correctamente", emisiones_por_calle)

        # 1. Descarga (usando el método de requests que vimos antes)
        print("Descargando datos de OSM")
        await websocket.send_json({"mensaje":"Descargando datos OSM"})


        if zonaSnachoFuerte==1:
            net_file = r"D:\Proyectos\SUMO_DOCKER\red_carreteras\zona-sancho-el-fuerte.net.xml"
            await websocket.send_json({"mensaje":"Red de sancho el fuerte descargada correctamente"})
        else:
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
                    "--remove-edges.isolated", "true",
                    "--tls.guess","true",
                    "--tls.join", "true"
                ], check=True)
            else:
                subprocess.run([
                    os.path.join(sumo_home, "bin", "netconvert"),
                    "--osm-files", osm_file,
                    "--output-file", net_file,
                    "--geometry.remove", "true",
                    "--proj.utm", "true",
                    "--keep-edges.by-type", sumo_types, # <--- Mantiene solo estos tipos
                    "--remove-edges.isolated", "true",
                    "--tls.guess","true",
                    "--tls.join", "true"
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
                "--fringe-factor", "10"
            ], check=True)  
        else:
            procesoTraffic=subprocess.run([
                "python", random_trips,
                "-n", net_file,
                "-r", route_file,
                "-e", str(duration_sec),  # Simular 3600 segundos de tráfico
                "--period", str(period), # Aparece un coche cada 0.5 segundos
                "--fringe-factor", "10"
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
                    <routing>
                        <device.rerouting.probability value="1.0"/>
                        <device.rerouting.period value="10"/>
                    </routing>
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
                # "--additional-files", "tipos_vehiculos.add.xml",
                "--device.rerouting.probability", "1.0", # Todos los coches pueden recalcular
                "--device.rerouting.period", "1",        # Recalcular en cuanto cambie algo
                "--device.rerouting.pre-period", "0",
                "--ignore-route-errors", "true",          # <--- ESTO EVITA QUE LA SIMULACIÓN SE PARE
                "--statistic-output", "stats.xml",   #muestra informacion al final de la simulacion
                "--tripinfo-output", "tripinfo.xml",   #muestra informacion al final de la simulacion
                "--duration-log.statistics", "true", # Esto saca un resumen rápido en la consola
                "--emission-output", "emisiones_por_calle.xml",   #muestra informacion al final de la simulacion
                "--emission-output.step-scaled", "true",
                "--no-step-log", "true"
            ])
            
            # CALLES
            edges=traci.edge.getIDList()
            print("Total de calles: ", len(edges))
            await websocket.send_json({"mensaje":"Total de calles: "+ str(len(edges))})

            lanes=traci.lane.getIDList()
            print("Total de carriles: ", len(lanes))
            await websocket.send_json({"mensaje":"Total de carriles: "+ str(len(lanes))})

            # PROHIBIR CALLES
            for edge_id in edges:
                if (edge_id in forbiddenRoadsArray):
                    traci.edge.setAllowed(edge_id, [])
                    traci.edge.setEffort(edge_id, 999999)
                    print("Calle prohibida: ", edge_id)
                    await websocket.send_json({"mensaje":"Calle prohibida: "+ edge_id})

            for lane_id in lanes:
                if (lane_id in forbiddenRoadsArray):
                    traci.lane.setAllowed(lane_id, [])
                    print("Carril prohibido: ", lane_id)
                    await websocket.send_json({"mensaje":"Carril prohibido: "+ lane_id})

            #SEMAFOROS
            # Ejecutar la simulación paso a paso
            step = 0
            await websocket.send_json({"simulationState":"1"})
            total_co2_mg = 0.0
            step_co2 = 0.0

            # Inicializar el diccionario de semáforos
            trafficLightDictionary = []
            net = sumolib.net.readNet(net_file, withInternal=True, withPedestrianConnections=True,withLatestPrograms=True)

            while step < duration_sec and traci.simulation.getMinExpectedNumber() > 0:
                try:
                    traci.simulationStep()  # Avanzar un paso

                    try:
                        # Intentamos leer un mensaje sin bloquear la simulación (timeout corto)
                        data = await asyncio.wait_for(websocket.receive_json(), timeout=0.01)
                        
                        if data.get("action") == "close_edge":
                            edge_id = data.get("edge_id")
                            await websocket.send_json({"calle_cerrada":"Cerrando calle " + edge_id})
                            try:
                                # Lógica de cierre en SUMO
                                # traci.edge.setAllowed(edge_id, [])  # Prohibir paso
                                # traci.edge.setEffort(edge_id, 999999) # Avisar al GPS
                            
                                traci.lane.setAllowed(edge_id, ["all"])  
                                traci.lane.setMaxSpeed(edge_id, 0.1)
                            
                                # Forzar rerouting a los coches que ya están en el mapa
                                # for veh_id in traci.vehicle.getIDList():
                                #     if traci.vehicle.getRoadID(veh_id) == edge_id:
                                #         traci.vehicle.rerouteTraveltime(veh_id)
                                await websocket.send_json({"calle_cerrada":"Calle cerrada correctamente " + edge_id})
                            except Exception as e:
                                await websocket.send_json({"calle_cerrada":"Error al cerrar la calle " + edge_id + " " + str(e)})
                            
                            # Confirmar al frontal
                            await websocket.send_json({"type": "status", "msg": f"Calle {edge_id} cerrada"})

                        if data.get("action") == "open_edge":
                            edge_id = data.get("edge_id")
                            await websocket.send_json({"calle_abierta":"Abriendo calle " + edge_id})
                            try:
                                # Lógica de cierre en SUMO
                                # traci.edge.setAllowed(edge_id, ["all"])  # Prohibir paso
                                # traci.edge.setEffort(edge_id, 1) # Avisar al GPS
                            
                                traci.lane.setAllowed(edge_id,  ["all"])  # Prohibir paso
                                traci.lane.setMaxSpeed(edge_id, 13.89) # Avisar al GPS
                            
                                # Forzar rerouting a los coches que ya están en el mapa
                                # for veh_id in traci.vehicle.getIDList():
                                #     if traci.vehicle.getRoadID(veh_id) == edge_id:
                                #         traci.vehicle.rerouteTraveltime(veh_id)

                                await websocket.send_json({"calle_abierta":"Calle abierta correctamente " + edge_id})
                            except Exception as e:
                                await websocket.send_json({"calle_abierta":"Error al abrir la calle " + edge_id + " " + str(e)})
                            
                            # Confirmar al frontal
                            await websocket.send_json({"type": "status", "msg": f"Calle {edge_id} abierta"})
                    
                    except asyncio.TimeoutError:
                        # No hay mensajes nuevos, seguimos la simulación
                        pass

                    
                    vehicles_at_step = []
                    for veh in traci.vehicle.getIDList():
                        traci.vehicle.rerouteTraveltime(veh)
                        # Obtener posición (x, y) en la proyección de SUMO
                        x, y = traci.vehicle.getPosition(veh)
                        step_co2 += traci.vehicle.getCO2Emission(veh)
                        
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

                        await websocket.send_json({"vehiculo":vehiculo})
                    
                    await asyncio.sleep(0.01)
                    # INSERCION DE SEMAFOROS
                    
                    lista_semaforos = traci.trafficlight.getIDList()
                    # # Semaforos
                    for tflID in traci.trafficlight.getIDList():
                        position = traci.junction.getPosition(tflID)
                        duration = traci.trafficlight.getPhaseDuration(tflID)
                        program = traci.trafficlight.getProgram(tflID)
                        links = traci.trafficlight.getControlledLinks(tflID)
                        lanes = traci.trafficlight.getControlledLanes(tflID)

                        lon, lat = traci.simulation.convertGeo(position[0], position[1])
                        state=traci.trafficlight.getRedYellowGreenState(tflID)

                        tfl ={
                            "id": tflID,
                            "longitude": lon,
                            "latitude": lat,
                            "state": state,
                            "color": getTrafficLightColor(state[0]),
                            "duration": duration,
                            "program": program,
                            "links": links,
                            "lanes": lanes  
                        }
                        await websocket.send_json({"trafficlight":tfl})

                    await asyncio.sleep(0.01)
                    
                    step += 1
                except traci.TraCIException as e:
                    if "has no valid route" in str(e):
                        print("Detectado error de ruta, saltando vehículo conflictivo...")
                        # El parámetro --ignore-route-errors en el start suele bastar,
                        # pero aquí podrías manejar lógica extra.
                

            
            total_co2_kg = total_co2_mg / 1000000
            print(f"Total CO2: {total_co2_kg} kg")
            await websocket.send_json({
               "final_report": {
                    "total_co2_kg": round(total_co2_kg, 2),
                    "equivalent_trees_day": round(total_co2_kg / 0.06, 2) # Un árbol absorbe aprox 60g/día
                }
            })

            # 1.1. Obtener emisiones por calle
            emisiones_por_calle = {}
            for edge_id in traci.edge.getIDList():
                emisiones_por_calle[edge_id] = traci.edge.getCO2Emission(edge_id)

            # 1.2. Calcular emisiones totales
            total_co2_mg = sum(emisiones_por_calle.values())
            total_co2_kg = total_co2_mg / 1_000_000

            # 1.3. Preparar estadísticas
            stats = {
                "vehiculos_totales": traci.simulation.getArrivedNumber() + traci.simulation.getMinExpectedNumber(),
                "emisiones_co2_actuales": total_co2_kg
            }
            print(f"Resumen de la simulación: {stats}")
            await websocket.send_json({"stats":stats})
            
            # 2. Leer el archivo de emisiones generado
            try:
                with open(emisiones_por_calle, 'r') as f:
                    # Leemos todo el contenido
                    contenido = f.read()
                    # Enviamos el contenido crudo al frontend
                    # (El frontend tendrá que parsear este XML)
                    await websocket.send_json({"emisiones_xml": contenido})
                    print("Archivo de emisiones enviado al frontend")
            except Exception as e:
                print(f"Error leyendo el archivo de emisiones: {e}")

            # Cerrar TraCI
            traci.close()
            print("Simulación finalizada correctamente")
            await websocket.send_json({"mensaje":"Simulación finalizada correctamente"})
            await websocket.send_json({"simulationState":"0"})
            await websocket.close()
            
                
            
            
        except Exception as e:
            print(f"Error en la simulación:")
            await websocket.send_json({"mensaje":"Error en la simulación: "+str(e)})
            if traci.isLoaded():
                traci.close()
            raise HTTPException(status_code=500, detail=str(e))

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

    bbox_str = websocket.query_params.get("mapa_")

    
    sumo_home = os.environ.get("SUMO_HOME")

    if not sumo_home:
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
                "--junctions.join", "true",
                "--proj.utm", "true",
                "--keep-edges.by-type", sumo_types, # <--- Mantiene solo estos tipos
                "--remove-edges.isolated", "true",
                "--output.street-names", "true",
                "--tls.guess","true",
                "--tls.join", "true"
            ], check=True)  
            print("Red de SUMO generada correctamente")
            
            # 3. FUNCION DE LEER LA RED GENERADA DE OSM
            net = sumolib.net.readNet(net_file, withInternal=True, withPedestrianConnections=True,withLatestPrograms=True)
            
            # INSERCION DE CARRETERAS
            for edge in net.getEdges():
                for lane in edge.getLanes():
                    # Obtenemos la geometría (forma) de la carretera
                    # Convertimos las coordenadas internas de SUMO a Lon/Lat
                    idLane=lane.getID()
                    shape = lane.getShape()
                    coords = [net.convertXY2LonLat(x, y) for x, y in shape]

                    velocityStyle = getVelocityStyle(lane.getSpeed() * 3.6)

                    # Extraemos las propiedades que queremos
                    # Nota: edge.getName() devuelve el nombre de la calle de OSM
                    print("Edge ID:", idLane)
                    properties = {
                        "type": "lane",
                        "id": idLane,
                        "nombre": edge.getName() or "Calle sin nombre",
                        "tipo": edge.getType(),
                        "velocidad_max": lane.getSpeed() * 3.6, # Convertir m/s a km/h
                        # "carriles": edge.getLaneNumber(),
                        "tamaño": lane.getLength(),
                        # "origen": edge,
                        # "destino": edge.getTo(),
                        # "prioridad": edge.getPriority(),    
                        "prohibida": False,
                        "color": velocityStyle,
                        # "orientation": lane.getAngle(lane.getEdgeID(),None),
                        "edgeID": edge.getID()
                    }

                    feature = {
                        "type": "feature",
                        "geometry": {
                            "type": "LineString",
                            "coordinates": coords
                        },
                        "properties": properties
                    }
                    await websocket.send_json(feature)
            await websocket.send_json({"mensaje": "Descarga de carreteras finalizada correctamente👍"})
            
            # INSERCION DE SEMAFOROS
            # Obtener todas uniones de la red entre lanes
            for junction in net.getNodes():
                idJunction =junction.getID()
                # Obtener el polígono de la unión
                shape = junction.getShape()
                coordinates = []
                for x, y in shape:
                    lon, lat = net.convertXY2LonLat(x, y)
                    coordinates.append([lon, lat])

                if coordinates:
                    coordinates.append(coordinates[0])

                feature = {
                    "type": "feature",
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [coordinates]
                    },
                    "properties": {
                        "id": idJunction,
                        "type": "junction",
                    }
                }
                
                await websocket.send_json(feature)
            await websocket.send_json({"mensaje": "Descarga de uniones finalizada correctamente👍"})

            # INSERCION DE SEMAFOROS
            semaforos_detallados = []

            for tls in net.getTrafficLights():
                tls_id = tls.getID()
                
                # El TLS nos da las conexiones (el "puente" entre calles)
                for connection in tls.getConnections():
                    # connection[0] es el carril de entrada (Lane)
                    lane_entrada = connection[0]
                    link_index = connection[2] # Su posición en el código de luces (0, 1, 2...)
                    
                    # El semáforo físico está al final del carril
                    shape = lane_entrada.getShape()
                    punto_final = shape[-1] 
                    
                    lon, lat = net.convertXY2LonLat(punto_final[0], punto_final[1])
                    
                    # Calculamos la orientación para que en Cesium no miren a Cuenca
                    # angulo = lane_entrada.getAngle(relativePos=-1)
                    
                    semaforos_detallados.append({
                        "id": f"{tls_id}_{link_index}",
                        "lon": lon,
                        "lat": lat,
                        # "heading": angulo
                    })

                    feature = {
                        "type": "feature",
                        "geometry": {
                            "type": "Point",
                            "coordinates": [lon, lat]
                        },
                        "properties": {
                            "id": f"{tls_id}_{link_index}",
                            "type": "trafficlight",
                        }
                    }
                    await websocket.send_json(feature)
            await websocket.send_json({"mensaje": "Descarga de semáforos finalizada correctamente👍 Nº:" + str(len(semaforos))})

            await websocket.close()
        except Exception as e:
            await websocket.send_json({"mensaje": "Error en la descarga de carreteras: "+str(e)})
            await websocket.close()
            raise HTTPException(status_code=500, detail=str(e))
        
        finally:
            await websocket.send_json({"mensaje": "Descarga de carreteras finalizada 👍"})
            await websocket.close()

@app.websocket("/ws/getRoadsPamplona")
async def get_roads_websocket(websocket: WebSocket):
    await websocket.accept()


    #DETERMINA EL SISTEMA OPERATIVO SOBRE EL QUE SE EJECUTA LA APLICACION
    operativeSytemIsLinux= 0 if platform.system()=="Linux" else 1
    if operativeSytemIsLinux==0:
       sumo_home = "/usr/share/sumo"
    else:
        sumo_home = r"C:\Program Files (x86)\Eclipse\Sumo"

    if not sumo_home:
        sumo_home = r"C:\Program Files (x86)\Eclipse\Sumo"
    
    net_file = ""

    print("Ruta de SUMO encontrada correctamente", sumo_home)
    with tempfile.TemporaryDirectory() as tmpdir:
        if operativeSytemIsLinux==0:
            net_file = os.path.join(tmpdir, "pamplona_v1.net.xml")
        else:
            net_file = os.path.join(r"D:\Proyectos\SUMO_DOCKER\red_carreteras", "zona-sancho-el-fuerte.net.xml")

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

