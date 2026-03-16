import sumolib
# import traci
import os
import sys
import subprocess
import tempfile
import requests
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional, List, Dict
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
from pathlib import Path
import pyarrow
import json

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

async def convert_net_to_geojson_net(websocket, net_file):
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

def parse_edge_data(xml_file: str) -> pd.DataFrame:
    tree = ET.parse(xml_file)
    root = tree.getroot()

    rows = []

    for interval in root.findall("interval"):
        begin = float(interval.attrib.get("begin", 0))
        end = float(interval.attrib.get("end", 0))

        for edge in interval.findall("edge"):
            row = {
                "begin": begin,
                "end": end,
                "edge_id": edge.attrib.get("id")
            }

            for key, value in edge.attrib.items():
                if key != "id":
                    row[key] = value

            rows.append(row)

    df = pd.DataFrame(rows)

    # Intentar convertir columnas numéricas automáticamente
    for col in df.columns:
        if col not in ["edge_id"]:
            df[col] = pd.to_numeric(df[col], errors="ignore")

    return df

def convertirEmissionsXmlToParquet(ruta_emissions,ruta_parquet,rootLabel='interval',nestLabel='edge'):
    try:
        tree = ET.parse(ruta_emissions)
        root = tree.getroot()

        lista_final = []

        # 2. Recorrer cada intervalo (el padre)
        for interval in root.findall(rootLabel):
            # Extraemos los datos del tiempo
            inicio = interval.get('begin')
            fin = interval.get('end')
            
            # 3. Recorrer cada edge dentro de ese intervalo (el hijo)
            for edge in interval.findall(nestLabel):
                # Copiamos todos los atributos del edge (id, CO2, fuel, etc.)
                datos_fila = edge.attrib.copy()
                
                # Añadimos la información del tiempo del padre a esta fila
                datos_fila['interval_begin'] = inicio
                datos_fila['interval_end'] = fin
                
                lista_final.append(datos_fila)

                # 4. Crear el DataFrame
        df = pd.DataFrame(lista_final)
        # 5. Limpieza de datos (Crucial para Cesium y análisis)
        # Convertimos a números lo que debe ser número
        cols_numericas = [c for c in df.columns if c not in ['id', 'interval_begin', 'interval_end']]
        for col in cols_numericas:
            df[col] = pd.to_numeric(df[col], errors='coerce')
        
        # Aseguramos que los tiempos también sean numéricos para filtrar en el mapa
        df['interval_begin'] = pd.to_numeric(df['interval_begin'])
        df['interval_end'] = pd.to_numeric(df['interval_end'])

        # 6. Guardar a Parquet
        # Mantenemos el 'id' intacto para que Cesium pueda hacer el JOIN con tu red .js o .geojson
        df.to_parquet(ruta_parquet, engine='pyarrow', index=False)
        
        print(f"Éxito: Se han procesado {len(df)} registros de edges.")

    except Exception as e:
        print(f"Error al ejecutar SUMO: {e}")
        raise HTTPException(status_code=500, detail=f"Error al ejecutar SUMO: {e}")

#---------------------------------------------------------------------------------------------------------


@app.get("/")
async def root():
    return {"status": "ok"}

@app.websocket("/ws/status")
async def websocket_status(websocket: WebSocket):
    try:
        await websocket.accept()
        await websocket.send_json({"mensaje": "Conexión WebSocket establecida correctamente"})

    except Exception as e:
        print(f"Error en WebSocket: {e}")
    finally:
        await websocket.close()

# ******************FUNCIONES DE PARSEO DE LOS RESULTADOS DE EMISIONES DE SUMO**********************#
def parse_sumo_emissions_edge(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    data = []

    for interval in root.findall('interval'):
        for edge in interval.findall('edge'):
            # Guardamos el ID y la métrica que nos interese (ej. CO2)
            entry = {
                'id': edge.get('id'),
                'co2': float(edge.get('CO2_abs')),
                'fuel': float(edge.get('fuel_abs'))
            }
            data.append(entry)
    
    return pd.DataFrame(data)

def parse_sumo_emissions_lane(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    data = []

    for interval in root.findall('interval'):
        for edge in interval.findall('edge'):
            for lane in edge.findall('lane'):
                # Guardamos el ID y la métrica que nos interese (ej. CO2)
                entry = {
                    'id': lane.get('id'),
                    'co2': float(lane.get('CO2_abs')),
                    'fuel': float(lane.get('fuel_abs'))
            }
            data.append(entry)
    
    return pd.DataFrame(data)

# ******************FIN FUNCIONES DE PARSEO DE LOS RESULTADOS DE EMISIONES DE SUMO**********************#

# RUTA PARA EJECUTAR LA SIMULACION DE SUMO Y OBTENER LOS RESULTADOS DE EMISIONES Y TRAFICO EN CALLES Y CARRILES
@app.get("/simulationEmissions")
async def simulationEmissions():
    # DETERMINA EL SISTEMA OPERATIVO SOBRE EL QUE SE EJECUTA LA APLICACION
    operativeSytemIsLinux= 1 if platform.system()=="Linux" else 0
    if operativeSytemIsLinux==1:
       sumo_home = "/usr/share/sumo"
       ruta_output= r"/tmp/twin-sumo-output/output"
    else:
       sumo_home = r"C:\Proyectos\01_sumo-1.26.0"
       ruta= r"C:\Proyectos\twin-sumo-output\red_carreteras"
       ruta_output= r"C:\Proyectos\twin-sumo-output\output"

    print("Ruta de SUMO encontrada correctamente", sumo_home,"Operative system",platform.system())

    # 1. Generar tráfico aleatorio sobre una red de ejemplo (sancho el fuerte)
    print("Generando tráfico aleatorio")
    
    with tempfile.TemporaryDirectory() as tmpdir:
        try:
            route_file = os.path.join(tmpdir, "mapa.rou.xml")
            print("Archivo ROUT creado correctamente", route_file)

            if operativeSytemIsLinux==1:
                net_file = "/tmp/zona-sancho-el-fuerte.net.xml"
                route_file= "/tmp/mapa.rou.xml"
            else:
                net_file =ruta + r"\zona-sancho-el-fuerte.net.xml"
                route_file= ruta + r"\mapa.rou.xml"

            random_trips = os.path.join(sumo_home, "tools", "randomTrips.py")
            if operativeSytemIsLinux==1:
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

            if operativeSytemIsLinux==1:
                config_file = "/tmp/simulation.sumocfg"
            else:
                config_file = ruta + r"\simulation.sumocfg"
                route_file = ruta + r"\mapa.rou.xml"
                
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

            if operativeSytemIsLinux==1:
                sumo = sumo_home
            else:
                sumo = os.path.join(sumo_home, "bin", "sumo")  # sin GUI
                
            print("Lanzando simulación con SUMO")
            try:
                subprocess.run([
                        sumo,
                        "-c", config_file,
                        "-b", "0",
                        "-e", "7200",
                        # "-n", net_file,
                        # "-r", route_file,
                        "-v", "true",
                        # "--full-output", full_output_file
                    ], check=True,capture_output=True, text=True)

                ruta_output= r"C:\Proyectos\twin-sumo-output\output"
                ruta_edgeEmissions = os.path.join(ruta_output, "edgeEmissions.xml")
                ruta_edgeEmissions_p = os.path.join(ruta_output, "edgeEmissions.parquet")
                convertirEmissionsXmlToParquet(ruta_edgeEmissions,ruta_edgeEmissions_p)
                print(" Fichero de edgeEmissions.parquet creado")
                


            except Exception as e:
                print(f"Error al ejecutar SUMO: {e}")
                raise HTTPException(status_code=500, detail=f"Error al ejecutar SUMO: {e}")
            finally:
                print("Finalizada la simulación con SUMO")

        except Exception as e:
            print(f"Error en la simulación: {e}")
            raise HTTPException(status_code=500, detail=f"Error en la simulación: {e}")
        
        finally:
            print("Finalizada la simulación con SUMO")




@app.websocket("/ws/getRoadsSanchoElFuerte")
async def getRoadsSanchoElFuerte(websocket: WebSocket):
    await websocket.accept()
    await websocket.send_json({"mensaje": "Iniciando descarga de carreteras de Sancho el Fuerte.🚩"})
    
    operativeSytemIsLinux= 0 if platform.system()=="Linux" else 1
    if operativeSytemIsLinux==0:
       sumo_home = "/usr/share/sumo"
    else:
        sumo_home = r"C:\Program Files (x86)\Eclipse\Sumo"

    print("Ruta de SUMO encontrada correctamente", sumo_home)
    await websocket.send_json({"mensaje": "Ruta de SUMO encontrada correctamente."+ sumo_home})
    if operativeSytemIsLinux==0:
        net_file = "/tmp/zona-sancho-el-fuerte.net.xml"
    else:
        net_file = "C:\\Proyectos\\SUMO_DOCKER\\red_carreteras\\zona-sancho-el-fuerte.net.xml"

    await websocket.send_json({"mensaje": "Iniciando descarga de red de Sancho el fuerte."})
    try:
        await websocket.send_json({"mensaje": "Iniciando envio calles Sancho el fuerte."})
        await convert_net_to_geojson_net(websocket,net_file)
        await websocket.send_json({"mensaje": "Iniciando envio calles Sancho el fuerte."})
    except Exception as e:
        await websocket.send_json({"mensaje": "Error al enviar calles de Sancho el fuerte."})
        await websocket.close()
        raise HTTPException(status_code=500, detail=str(e))
    else:
        await websocket.send_json({"mensaje": "Envío de calles de Sancho el fuerte finalizado correctamente.👍"})
    
    finally:
        await websocket.close()


def generar_czml(parquet_path, czml_path):
    df = pd.read_parquet(parquet_path)
    
    # 1. El 'Document' es obligatorio en CZML
    czml = [{
        "id": "document",
        "name": "Simulacion SUMO",
        "version": "1.0"
    }]

    # 2. Agrupamos por edge para crear una línea de tiempo por cada calle
    for edge_id, group in df.groupby('id'):
        
        # Creamos el paquete para este edge específico
        packet = {
            "id": str(edge_id),
            "name": f"Edge {edge_id}",
            "polyline": {
                "width": 5,
                "material": {
                    "solidColor": {
                        "color": {
                            # Aquí definimos cómo cambia el color con el tiempo
                            "rgba": [] 
                        }
                    }
                }
            }
        }

        # 3. Llenamos la línea de tiempo (emisiones o densidad)
        for fila in group.itertuples():
            # Convertimos el tiempo de SUMO (segundos) a formato ISO8601 de Cesium
            # Ej: "2023-10-27T10:00:00Z"
            tiempo_iso = f"2026-03-13T00:00:{int(fila.interval_begin):02d}Z"
            
            # Ejemplo: Si el CO2 es alto, ponemos color rojo (255, 0, 0)
            color = [255, 0, 0, 255] if fila.CO2_abs > 500 else [0, 255, 0, 255]
            
            packet["polyline"]["material"]["solidColor"]["color"]["rgba"].extend(
                [tiempo_iso] + color
            )

        czml.append(packet)

    # 4. Guardar el resultado
    with open(czml_path, "w") as f:
        json.dump(czml, f)

