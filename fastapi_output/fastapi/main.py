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
from sumolib import checkBinary
import traci
from urllib.parse import unquote
import platform
import asyncio
from pyproj import Geod
import math
import xml.etree.ElementTree as ET
import pandas as pd
sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

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
    await websocket.send_json({"mensaje":"Ruta de SUMO encontrada correctamente"+ sumo_home +"| Operative system:"+platform.system()})

    with tempfile.TemporaryDirectory() as tmpdir:
        print("Directorio temporal creado correctamente", tmpdir)
        osm_file = os.path.join(tmpdir, "mapa.osm.xml")
        print("Archivo OSM creado correctamente", osm_file)
        net_file = os.path.join(tmpdir, "mapa.net.xml")
        print("Archivo NET creado correctamente", net_file)
        route_file = os.path.join(tmpdir, "mapa.rou.xml")
        print("Archivo ROUT creado correctamente", route_file)
        type_vehicles_file=os.path.join(tmpdir,"tipos_vehiculos.add.xml")
        print("Tipos de vehiculos", type_vehicles_file)

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
            if operativeSytemIsLinux==0:
                net_file = "/tmp/zona-sancho-el-fuerte.net.xml"
            else:
                net_file = "D:\\Proyectos\\SUMO_DOCKER\\red_carreteras\\zona-sancho-el-fuerte.net.xml"
            
            sumo_types = ",".join([f"highway.{t}" for t in ["motorway", "motorway_link","motorway_junction", "primary", "secondary", "tertiary", "residential", "living_street","trunk","trunk_link", "primary_link", "secondary_link", "tertiary_link","service","trafficlight"]])
            await websocket.send_json({"mensaje":"Red de sancho el fuerte descargada correctamente"})
        else:
            download_osm_data(bbox, osm_file)
            print("Datos de OSM descargados correctamente")
            await websocket.send_json({"mensaje":"Datos de OSM descargados correctamente"})

            sumo_types = ",".join([f"highway.{t}" for t in bbox.road_types])
        
            # 2. Generar red de SUMO
            print("Generando red de SUMO")
            await websocket.send_json({"mensaje":"Generando red de SUMO de OpenStreetMap"})
        
        if zonaSnachoFuerte==0:
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
        if zonaSnachoFuerte==0:
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
                        <output>
                            <tripinfo-output value="tripinfos.xml"/>
                        </output>
                        <routing>
                            <device.rerouting.probability value="1.0"/>
                            <device.rerouting.period value="10"/>
                        </routing>
                    </configuration>""")

            

            print("Archivos de configuración SUMO generados correctamente")
            config_file = os.path.join(tmpdir, "simulation.sumocfg")
        if zonaSnachoFuerte==1:
            if operativeSytemIsLinux==1:
                config_file = os.path.join("../adicionalFiles/sancho-el-fuerte.sumocfg")
            else:
                config_file = os.path.join("/tmp/sancho-el-fuerte.sumocfg")
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
                # "--duration-log.statistics", "true", # Esto saca un resumen rápido en la consola
                "--emission-output.geo","true",
                "--emission-output", "emission.xml",   #muestra informacion al final de la simulacion
                "--emission-output.step-scaled", "true",
                # "--no-step-log", "true"
                "--summary-output", "summary.xml",

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
                        data = await asyncio.wait_for(websocket.receive_json(), timeout=0.02)

                        if (data.get("action")=="insert_flow"):
                            numberOfCars=int(data.get("numberOfCars"))
                            origin=data.get("origin")
                            destination = data.get("destination")
                            route_calculada = findRoute(traci,data.get("origin"),data.get("destination"))
                            uuid_str=str(uuid.uuid4().int)
                            route_id = "route_" + uuid_str
                            traci.route.add(route_id, route_calculada.edges)
                            print(f"Creada ruta {route_id} de {origin} a {destination}")
                            i=0
                            while i<numberOfCars:
                                uuid_vehicle=str(uuid.uuid4().int)
                                traci.vehicle.add(uuid_vehicle,route_id)
                                traci.vehicle.rerouteTraveltime(uuid_vehicle)
                                print(f"Vehículo {uuid_vehicle} insertador en ruta {route_id}")
                                await websocket.send_json({"vehicle_inyect":"Vehículo "+uuid_vehicle +" insertado en ruta "+ route_id})
                                i+=1
                            
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
                    # Esto hace que los vehiculos que han terminado la ruta desaparezcan
                    for vehicleID in traci.simulation.getArrivedIDList():
                        vehiculo={
                            "id": vehicleID
                        }
                        await websocket.send_json({"vehiculo_finalizado":vehiculo})

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
                    
                    # Esto hace que los vehiculos que han terminado la ruta desaparezcan
                    for vehicleID in traci.simulation.getArrivedIDList():
                        vehiculo={
                            "id": vehicleID
                        }
                        await websocket.send_json({"vehiculo_finalizado":vehiculo})

                    await asyncio.sleep(0.01)
                    # INSERCION DE SEMAFOROS
                    
                    lista_semaforos = traci.trafficlight.getIDList()
                    # # Semaforos
                    for tflID in traci.trafficlight.getIDList():
                        position=None
                        if tflID.startswith("GS_"):
                            position = traci.junction.getPosition(tflID[3:len(tflID)])
                        else:
                            position = traci.junction.getPosition(tflID)
                        
                        # programs = traci.trafficlight.getAllProgramLogics(tflID)
                        lon, lat = traci.simulation.convertGeo(position[0], position[1])
                        state=traci.trafficlight.getRedYellowGreenState(tflID)

                        tfl ={
                            "id": tflID,
                            "longitude": lon,
                            "latitude": lat,
                            "state": state,
                            "color": getTrafficLightColor(state[0]),
                            # "programs": programs
                        }
                        await websocket.send_json({"trafficlight":tfl})

                    await asyncio.sleep(0.01)
                    
                    step += 1
                    await websocket.send_json({"step": step})
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

            tree = ET.parse("tripinfo.xml")
            root = tree.getroot()
            data = []
            for trip in root.findall("tripinfo"):
                data.append({
                    "id": trip.get("id"),
                    "duration": float(trip.get("duration")),
                    "waitingTime": float(trip.get("waitingTime")),
                    "timeLoss": float(trip.get("timeLoss"))
                })

            df = pd.DataFrame(data)
            
            print("Duración media:", df["duration"].mean())
            print("Tiempo de espera medio:", df["waitingTime"].mean())
            await websocket.send_json({"stats":"Duración media:"+ str(df["duration"].mean())})
            await websocket.send_json({"stats":"Tiempo de espera medio:"+ str(df["waitingTime"].mean())})
            
            await websocket.close()
            
                
            
            
        except Exception as e:
            print(f"Error en la simulación:")
            await websocket.send_json({"mensaje":"Error en la simulación: "+str(e)})
            if traci.isLoaded():
                traci.close()
            raise HTTPException(status_code=500, detail=str(e))


#**************************SIMULACION******************************#
@app.get("/simulation_output")
async def simulacion_output():
    
    # DETERMINA EL SISTEMA OPERATIVO SOBRE EL QUE SE EJECUTA LA APLICACION
    operativeSytemIsLinux= 0 if platform.system()=="Linux" else 1
    if operativeSytemIsLinux==0:
       sumo_home = "/usr/share/sumo"
    else:
       sumo_home = r"C:\Proyectos\01_sumo-1.26.0"
       ruta= r"C:\Proyectos\twin-sumo-output\red_carreteras"
       ruta_output= r"C:\Proyectos\twin-sumo-output\output_files"

    print("Ruta de SUMO encontrada correctamente", sumo_home,"Operative system",platform.system())

    # 1. Generar tráfico aleatorio sobre una red de ejemplo (sancho el fuerte)
    print("Generando tráfico aleatorio")
    with tempfile.TemporaryDirectory() as tmpdir:
        try:
            route_file = os.path.join(tmpdir, "mapa.rou.xml")
            print("Archivo ROUT creado correctamente", route_file)

            if operativeSytemIsLinux==0:
                net_file = "/tmp/zona-sancho-el-fuerte.net.xml"
                route_file= "/tmp/mapa.rou.xml"
            else:
                net_file =ruta + r"\zona-sancho-el-fuerte.net.xml"
                route_file= ruta + r"\mapa.rou.xml"

            random_trips = os.path.join(sumo_home, "tools", "randomTrips.py")
            if operativeSytemIsLinux==0:
                subprocess.run([
                    "python3", random_trips,
                    "-n", net_file,
                    "-r", route_file,
                    "-e", "3600",  # Simular 3600 segundos de tráfico
                    "--period", "10", # Aparece un coche cada 0.5 segundos
                    "--fringe-factor", "10"
                ], check=True)  
            else:
                subprocess.run([
                    "python", random_trips,
                    "-n", net_file,
                    "-r", route_file,
                    "-e", "3600",  # Simular 3600 segundos de tráfico
                    "--period", "10", # Aparece un coche cada 0.5 segundos
                    "--fringe-factor", "10"
                ], check=True)  

            # crea el archivo de configuración SUMO

            if operativeSytemIsLinux==0:
                config_file = "/tmp/simulation.sumocfg"
            else:
                config_file = ruta + r"\simulation.sumocfg"
                route_file = ruta + r"\mapa.rou.xml"
                # edgeDataTraffic_file = ruta_output + r"\edgeDataTraffic.xml" # ESTADISTICAS DE TRAFICO DE CADA CALLE
                # laneDataTraffic_file = ruta_output + r"\laneDataTraffic.xml"
                # full_output_file = ruta_output + r"\full_output.xml"
                # net_state_file = ruta_output + r"\net_state.xml"
                # stats_file = ruta_output + r"\stats.xml"
                # edgeDataEmissions_file = ruta_output + r"\edgeDataEmissions.xml"
                # laneDataEmissions_file = ruta_output + r"\laneDataEmissions.xml"

                # vehicle_emissions_file = ruta_output + r"\vehicle_emissions.xml"
                
                
                # summary_file = ruta_output + r"\summary.xml"
                # tripinfo_file = ruta_output + r"\tripinfo.xml"
                # mapa_estadisticas_file = ruta_output + r"\mapa_estadisticas.xml"
                # edge_emissions_file = ruta_output + r"\edge_emissions.xml"
                # datos_calles_file = ruta_output + r"\datos_calles.xml"


            print("Archivo de configuración SUMO creado correctamente", config_file)
            with open(config_file, 'w') as f:
                f.write(f"""<?xml version="1.0" encoding="UTF-8"?>
                <configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.xsd">
                    <input>
                        <net-file value="zona-sancho-el-fuerte.net.xml"/>
                        <route-files value="{route_file}"/>
                        <additional-files value="additional.add.xml"/>
                    </input>
                    <routing>
                        <device.rerouting.probability value="1.0"/>
                        <device.rerouting.period value="10"/>
                    </routing>
                </configuration>""")

            print("Archivos de configuración SUMO generados correctamente")

            sumo = os.path.join(sumo_home, "bin", "sumo")  #
            print("Lanzando simulación con SUMO")
            try:
                subprocess.run([
                        sumo,
                        "-c", config_file,
                        "-b", "0",
                        "-e", "3600",
                        # "-n", net_file,
                        # "-r", route_file,
                        "-v", "true"
                    ], check=True,capture_output=True, text=True)  

                print("Finalizada la simulación con SUMO")
                
                # xml2csv = os.path.join(sumo_home, "tools","xml", "xml2csv.py")
                # subprocess.run([
                #     "python", xml2csv, "../output/edgeEmissions.xml"
                #     "--output", "../output/edgeEmissions.csv"
                #     ], check=True)  


                return {"mensaje": "Simulación finalizada correctamente"}
            except Exception as e:
                print(f"Error al ejecutar SUMO: {e}")
                raise HTTPException(status_code=500, detail=f"Error al ejecutar SUMO: {e}")
        
        except Exception as e:
            print(f"Error en la simulación: {e}")
            raise HTTPException(status_code=500, detail=f"Error en la simulación: {e}")d
            

