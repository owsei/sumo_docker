
        // const endPoint= "ws://owsei.ddns.net:8001/"
        // const endPoint= "http://localhost:8000/"
        // const endPointDirecto="ws://localhost:8000/"

        // Reemplaza con tu token de Cesium Ion
       

        // variables para el dibujo del rectangulo
        let firstPoint = null;
        let firstPointPosition = null;
        let secondPointPosition = null;
        let secondPoint = null;
        let rectangleEntity = null;
        let bounds = null;

        // variables para la comunicacion con el servidor
        let socket = null;
        let simulacionIniciada = false;

        // funciones para el movimiento de la camara
        function flyToA10(){
            window.viewer.camera.flyTo({
                destination: Cesium.Cartesian3.fromDegrees(-1.919479,42.913741, 13000),
                duration: 0
            });
        }
        
        // funcion para volar a pamplona
        function flyToPamplona(){
            window.viewer.camera.flyTo({
                destination: Cesium.Cartesian3.fromDegrees(-1.652673,42.816455, 13000),
                duration: 0
            });
        }


        let isDragging = false;
        let startPosition = null;
        let endPosition = null;
        let floatingRectangle = null;

        let vehicleEntities = {};
        let roadsEntities = {};

        let forbiddenRoadsArray =[];

        let simulationState = 0; // 0 = stopped, 1 = running

        const roadStyles = {
            'highway.motorway': { color: Cesium.Color.BLUE, width: 4 },
            'highway.primary': { color: Cesium.Color.GREEN, width: 4 },
            'highway.secondary': { color: Cesium.Color.YELLOW, width: 4 },
            'highway.tertiary': { color: Cesium.Color.PURPLE, width: 4 },
            'highway.trunk': { color: Cesium.Color.ORANGE, width: 4 },
            'highway.residential': { color: Cesium.Color.WHITE, width: 4 },
            'default': { color: Cesium.Color.GRAY, width: 4 }
        };

        const velocityStyles = {
            '120': { color: Cesium.Color.BLUE, width: 4 },
            '100': { color: Cesium.Color.GREEN, width: 4 },
            '80': { color: Cesium.Color.YELLOW, width: 4 },
            '60': { color: Cesium.Color.PURPLE, width: 4 },
            '40': { color: Cesium.Color.ORANGE, width: 4 },
            '20': { color: Cesium.Color.WHITE, width: 4 },
            'default': { color: Cesium.Color.GRAY, width: 4 }
        };

        function mostrarSpinner(){
            document.getElementById('spinnerOverlay').style.display = 'block';
        }

        function ocultarSpinner(){
            document.getElementById('spinnerOverlay').style.display = 'none';
        }

        function closeEdgeWebsocket(idEdge) {
            const comando = {
                action: "close_edge",
                edge_id: idEdge
            };
            
            socket.send(JSON.stringify(comando));
            console.log("Enviando petición de cierre para:", idEdge);

        }

        function setForbiddenRoads(idRoad,nameRoad){
            forbiddenRoadsArray.push({id:idRoad,nombreCalle:nameRoad});
            htmlForbbidenRoads=''
            forbiddenRoadsArray.forEach(element => {
                htmlForbbidenRoads+=`<div class="alert alert-danger" role="alert">${element.nombreCalle} - ${element.id} <button onclick="enableForbiddenRoad('${element.nombreCalle}','${element.id}')" class="btn btn-primary" id="enableForbiddenRoadBtn">Habilitar👍</button></div>`;
            });
            document.getElementById('forbiddenRoads').innerHTML = htmlForbbidenRoads;

            if (simulacionIniciada){
                closeEdgeWebsocket(idRoad);
            }
        }

        function openEdgeWebsocket(idEdge) {
            const comando = {
                action: "open_edge",
                edge_id: idEdge
            };
            
            socket.send(JSON.stringify(comando));
            console.log("Enviando petición de apertura para:", idEdge);

        }

        function removeItemOnce(arr, value) {
            return arr.filter(item => item.id !== value.id);
        }

        function enableForbiddenRoad(nombreCalle,idEdge){
            forbiddenRoadsArray = removeItemOnce(forbiddenRoadsArray, {id:idEdge,nombreCalle:nombreCalle});
            document.getElementById('forbiddenRoads').innerHTML = '';
            forbiddenRoadsArray.forEach(element => {
                document.getElementById('forbiddenRoads').innerHTML+=`<div class="alert alert-danger" role="alert">${element.nombreCalle} - ${element} <button onclick="enableForbiddenRoad('${element.nombreCalle}','${element.id}')" class="btn btn-primary" id="enableForbiddenRoadBtn">Habilitar👍</button></div>`;
            });
            
            let entidad = window.viewer.entities.values.filter(e => e.properties && e.properties.id && e.properties.id.getValue() === idEdge);
            console.log(entidad);
            if (entidad.length>0) {
                let color=entidad[0].properties.color.getValue();
                entidad[0].polyline.material.color = color;
            }
            if (simulacionIniciada){
                openEdgeWebsocket(idEdge);
            }
        }

        function getVelocityStyle(velocity) {

            if (velocity>119) {
                return Cesium.Color.BLUE;
            } else if (velocity>101 && velocity<=119) {
                return Cesium.Color.GREEN;
            } else if (velocity>80 && velocity<=101) {
                return Cesium.Color.YELLOW;
            } else if (velocity>60 && velocity<=80) {
                return Cesium.Color.PURPLE;
            } else if (velocity>40 && velocity<=60) {
                return Cesium.Color.ORANGE;
            } else if (velocity>20 && velocity<=40) {
                return Cesium.Color.WHITE;
            } else {
                return Cesium.Color.GRAY;
            }
        }

        function createPropertiesPanel(properties) {
    
            let descrip = `<table id="tablaProperties${properties.id}"  class="tablePorperties">`;
            for (const [key, value] of Object.entries(properties)) {
                if (key === 'enlace'|| key === 'url' || key === 'link' || key === 'source') {
                    // Si el valor es un enlace, lo formateamos como un enlace HTML
                    descrip = descrip + `<tr class="trproper">
                                            <td>Enlace</td>
                                            <td class="tdproper">
                                            <a href="${value}" target="_blank">Ir a Catastro</a>
                                            </td>
                                        </tr>`;
                }
                else{
                    descrip = descrip + `<tr class="trproper">
                                            <td class="tdproper">${key}</td>
                                            <td class="tdproper"> ${value}</td>
                                        </tr>`;
                }
                
            }
            descrip = descrip + `<tr><td> <button id="${properties.id}" class="cesium-button">
                                Prohibir ⛔
                            </button></td></tr>`;
            descrip = descrip + '</table>';

            return descrip;
        }

        function messageWebsocket(data){
            if (data.mensaje ) {
                console.log('Mensaje de prueba:', data);
                document.getElementById('messages-websocket').style.display = 'block';
                document.getElementById('messages-websocket').style.color = data.color;
                document.getElementById('messages-websocket').innerHTML += data.mensaje + "<br/>";
                return;
            }
            console.log('Mensaje de prueba:', data);
        }

        function crearLaneString(datosLineString,altura=0) {
            const coordenadas = datosLineString.geometry.coordinates; 
            const posiciones = coordenadas.map(coord => {
            const [lon, lat] = coord;
            return Cesium.Cartesian3.fromDegrees(lon, lat, 0);
            });

            const velocityStyle = getVelocityStyle(datosLineString.properties.velocidad_max);

            
            window.viewer.entities.add({
            properties: {
                id: datosLineString.properties.id,
                velocidad_max: datosLineString.properties.velocidad_max,
                nombre: datosLineString.properties.nombre,
                carriles: datosLineString.properties.carriles,
                prohibida: datosLineString.properties.prohibida,
                tamaño: datosLineString.properties.tamaño,
                origen: datosLineString.properties.origen,
                destino: datosLineString.properties.destino,
                prioridad: datosLineString.properties.prioridad,
                color: datosLineString.properties.color

            },
            polyline: {
                positions: posiciones,
                material : Cesium.Color.fromCssColorString(datosLineString.properties.color?datosLineString.properties.color:"white"),
                width: 3,
                clampToGround: true, // Para que la línea se ajuste al terreno
                heightReference: Cesium.HeightReference.RELATIVE_TO_GROUND,
                disableDepthTestDistance: Number.POSITIVE_INFINITY,
            },
            description: `
                        <div id="tablaProperties${datosLineString.properties.id}"  class="tablePorperties" >
                            <p>Información de la entidad</p>
                            <p>ID: ${datosLineString.properties.id}</p>
                            <p>Carriles: ${datosLineString.properties.carriles}</p>
                            <p>Calle: ${datosLineString.properties.nombre}</p>
                            <p>Velocidad máxima: ${datosLineString.properties.velocidad_max}</p>
                            <p>Prohibida: ${datosLineString.properties.prohibida}</p>
                            <p>Color: ${datosLineString.properties.color}</p>
                            <p>Tamaño: ${datosLineString.properties.tamaño}</p>
                            <p>Origen: ${datosLineString.properties.origen}</p>
                            <p>Destino: ${datosLineString.properties.destino}</p>
                            <p>Prioridad: ${datosLineString.properties.prioridad}</p>
                            <button id="${datosLineString.properties.id}" class="cesium-button">
                                Prohibir ⛔
                            </button>
                        </div>
                    `,
            });
        }

        function createJunction(feature){   
            const coordenadas = feature.geometry.coordinates; 
            
            window.viewer.entities.add({
            properties: {
                id: feature.properties.id,
                tipo: feature.properties.tipo,
            },
            polygon: {
                hierarchy: new Cesium.PolygonHierarchy(coordenadas),
                material: Cesium.Color.fromCssColorString("darkred"),
                outline: true,
                outlineColor: Cesium.Color.BLACK,
                clampToGround: true, // Para que el polígono se ajuste al terreno
                heightReference: Cesium.HeightReference.RELATIVE_TO_GROUND, // Úsalo si el polígono está en terreno
                disableDepthTestDistance: Number.POSITIVE_INFINITY,
                
            },
            description: `
                        <div id="tablaProperties${feature.properties.id}"  class="tablePorperties" >
                            <p>Información de la entidad</p>
                            <p>ID: ${feature.properties.id}</p>
                            <p>Tipo: ${feature.properties.tipo}</p>
                        </div>
                    `,
            });
        }

        function createTrafficLight(feature){
            const coordenadas = feature.geometry.coordinates;
            const [lon, lat] = coordenadas;
            console.log('Coordenadas:', coordenadas);
            console.log('Lat:', lat, 'Lon:', lon);


            // Crear la posición en el mundo
            const position = Cesium.Cartesian3.fromDegrees(lon, lat);


            window.viewer.entities.add({
                position: position,
                // orientation: orientation,
                // model: { uri: '3DObjets/semaforo.glb' },
                material : Cesium.Color.fromCssColorString("red"),
                point: {
                    pixelSize: 10,
                    outlineColor: Cesium.Color.BLACK,
                    clampToGround: true, // Para que el polígono se ajuste al terreno
                    heightReference: Cesium.HeightReference.RELATIVE_TO_GROUND, // Úsalo si el polígono está en terreno
                    disableDepthTestDistance: Number.POSITIVE_INFINITY,
                },
                description: `
                        <div id="tablaProperties${feature.properties.id}"  class="tablePorperties" >
                            <p>Información de la entidad</p>
                            <p>ID: ${feature.properties.id}</p>
                            <p>Tipo: ${feature.properties.type}</p>

                        </div>
                    `
            });
        }


        // LLAMADAS A LA APIs
        // WEBSOCKET
        // Test websocket
        async function getRoadsWebsocket(){
            if (!bounds) return;
            // Eliminar el rectángulo de la selección
            window.viewer.entities.remove(rectangleEntity);
            document.getElementById('messages-websocket').innerHTML += "Conectando al servidor...<br/>";
            const payload = {
                west: Cesium.Math.toDegrees(bounds.west),
                south: Cesium.Math.toDegrees(bounds.south),
                east: Cesium.Math.toDegrees(bounds.east),
                north: Cesium.Math.toDegrees(bounds.north),
                // road_types: ["motorway", "motorway_link", "primary", "secondary", "tertiary", "residential", "unclassified", "living_street", "service", "footway", "cycleway", "steps", "path", "track", "bridleway", "corridor", "ferry", "construction", "proposed", "raceway", "platform", "elevator", "escalator", "moving_walkway", "transit", "subway", "tram", "light_rail", "rail", "monorail"]    
                road_types: ["motorway", "motorway_link","motorway_junction", "primary", "secondary", "tertiary", "residential", "living_street","trunk","trunk_link", "primary_link", "secondary_link", "tertiary_link","service","trafficlight"]    
                
            };

            socket = new WebSocket(window.endPoint+'ws/getRoads?bbox='+JSON.stringify(payload));
            socket.onopen = () => {
                console.log('Conectado al servidor');
                document.getElementById('messages-websocket').innerHTML += "Conectando al servidor...";
            };

            socket.onmessage = function(event) {
                const data = JSON.parse(event.data);
                if (data.mensaje){
                    messageWebsocket(data);
                    return;
                }
                console.log(data);
                if (data.type=="feature"){
                    if(data.properties.type=="edge" || data.properties.type=="lane"){
                        crearLaneString(data);
                        return;
                    }
                    
                    if(data.properties.type=="junction"){
                        createJunction(data);
                        return;
                    }

                    if(data.properties.type=="trafficlight"){
                        createTrafficLight(data);
                        return;
                    }
                }
            }
        }   

        async function getRoadsPamplona(){

            const socket = new WebSocket(window.endPoint+'ws/getRoadsPamplona');
            socket.onopen = () => {
                console.log('Conectado al servidor');
            };

            socket.onmessage = function(event) {
                const data = JSON.parse(event.data);
                if (data.mensaje){
                    messageWebsocket(data);
                    return;
                }
                
                crearLaneString(data);
            }
        }

        // Run simulation websocket
        async function runSimulationWebsocket() {
            if (!bounds && window.zonaSnachoFuerte==0) return;
            // Eliminar el rectángulo de la selección
            window.viewer.entities.remove(rectangleEntity);
            document.getElementById('messages-websocket').innerHTML += "Conectando al servidor...<br/>";
            let payload = {};
            if (window.zonaSnachoFuerte==1){
                payload = {
                    
                }
            }else{
                payload = {
                    west: Cesium.Math.toDegrees(bounds.west),
                    south: Cesium.Math.toDegrees(bounds.south),
                    east: Cesium.Math.toDegrees(bounds.east),
                    north: Cesium.Math.toDegrees(bounds.north),
                    // road_types: ["motorway", "motorway_link", "primary", "secondary", "tertiary", "residential", "unclassified", "living_street", "service", "footway", "cycleway", "steps", "path", "track", "bridleway", "corridor", "ferry", "construction", "proposed", "raceway", "platform", "elevator", "escalator", "moving_walkway", "transit", "subway", "tram", "light_rail", "rail", "monorail"]    
                    road_types: ["motorway", "motorway_link","motorway_junction", "primary", "secondary", "tertiary", "residential", "living_street","trunk","trunk_link", "primary_link", "secondary_link", "tertiary_link","service","traffic_signals","crossing"]
                };
            }

            const encoded = forbiddenRoadsArray.map(road => road.id.replace(/#/g, '%23'));
            const forbiddenRoads = JSON.stringify(encoded);

            const num_vehicles = document.getElementById('num_vehicles').value;
            const duration_sec = document.getElementById('duration_sec').value;

            socket = new WebSocket(window.endPoint+'ws/simulation?bbox='+JSON.stringify(payload)+"&forbiddenRoads="+forbiddenRoads+"&num_vehicles="+num_vehicles+"&duration_sec="+duration_sec+"&zonaSnachoFuerte="+window.zonaSnachoFuerte);
            window.vehicles = {}; // Diccionario para rastrear entidades
            window.trafficLights = {}; // Diccionario para rastrear entidades

            socket.onopen = () => {
                console.log('Conectado al servidor');
            };
    
            socket.onmessage = function(event) {
                let data = JSON.parse(event.data);
                if (data.time==575 && data.id=="9"){
                    console.log(data);
                }
                if (data.mensaje ) {
                    document.getElementById('messages-websocket').innerHTML += data.mensaje + "<br/>";
                    return;
                }

                if (data.calle_cerrada ) {
                    document.getElementById('messages-websocket').innerHTML += data.calle_cerrada + "<br/>";
                    return;
                }

                if (data.calle_abierta ) {
                    document.getElementById('messages-websocket').innerHTML += data.calle_abierta + "<br/>";
                    
                    return;
                }

                // 0 = stopped, 1 = running
                if (data.simulationState)
                {
                    console.log('Estado de la simulación:', data.simulationState);
                    if (data.simulationState == "0") {
                        simulacionIniciada = false;
                    } else if (data.simulationState == "1") {
                        simulacionIniciada = true;
                    }
                    return;
                }
                

                if (data.trafficlight){
                    data = data.trafficlight;

                    console.log('Datos de semáforos:', data);
                    const position = Cesium.Cartesian3.fromDegrees(data.longitude, data.latitude);

                    if (!window.trafficLights.hasOwnProperty(data.id)) {
                        // Crear el semáforo si no existe
                        window.trafficLights[data.id] = window.viewer.entities.add({
                            id: data.id,
                            position: position,
                            description: createPropertiesPanel(data),
                            point: {
                                pixelSize: 10,
                                color: Cesium.Color.fromCssColorString(data.color)
                            },
                            label: { 
                                text: data.id, 
                                font: '10pt sans-serif',
                                fillColor: Cesium.Color.WHITE,
                                outlineColor: Cesium.Color.BLACK,
                                outlineWidth: 2,
                                verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
                                pixelOffset: new Cesium.Cartesian2(0, -20) 
                            }
                        });
                    } else {
                        // Actualizar posición y rotación en tiempo real
                        window.viewer.entities.getById(data.id).point.color.setValue(Cesium.Color.fromCssColorString(data.color));
                    }
                }   


                if (data.vehiculo){
                    data = data.vehiculo;
                
                    console.log('Datos de vehículos:', data);
                    const position = Cesium.Cartesian3.fromDegrees(data.longitude, data.latitude);

                    if (!vehicles.hasOwnProperty(data.id)) {
                        // Crear el vehículo si no existe
                        const orientation = Cesium.Transforms.headingPitchRollQuaternion(
                            position,
                            new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(data.angle), 0, 0)
                        );

                        const random_number = Math.floor(Math.random() * 7) + 1;
                        let URI_Coche;
                        if (random_number === 1) {
                            URI_Coche = "../3dobjects/coches/forales.glb";
                        } else if (random_number === 2) {
                            URI_Coche = "../3dobjects/coches/cocheAzulCubo.glb";
                        } else if (random_number === 3) {
                            URI_Coche = "../3dobjects/coches/cocheVerdeCubo.glb";
                        } else if (random_number === 4) {
                            URI_Coche = "../3dobjects/coches/cocheRojoCubo.glb";
                        } else if (random_number === 5) {
                            URI_Coche = "../3dobjects/coches/cocheAmarilloCubo.glb";
                        } else if (random_number === 6) {
                            URI_Coche = "../3dobjects/coches/cocheMoradoCubo.glb";
                        } else if (random_number === 7) {
                            URI_Coche = "../3dobjects/coches/cocheNegroCubo.glb"
                        }

                        

                        vehicles[data.id] = window.viewer.entities.add({
                            id: data.id,
                            position: position,
                            orientation: orientation,
                            description: createPropertiesPanel(data),
                            // Orientación basada en el ángulo de SUMO
                            // orientation: Cesium.Transforms.headingPitchRollQuaternion(
                            //     position,
                            //     new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(data.angle), 0, 0)
                            // ),
                            model: {
                                uri: URI_Coche, // Ruta a tu carpeta
                                minimumPixelSize: 20,          // Para que no desaparezca al alejarse
                                scale: 20.0,                  // Ajusta según el tamaño de tu .glb
                                heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
                                clampToGround: true,
                                disableDepthTestDistance: Number.POSITIVE_INFINITY, 
                            },
                            label: { 
                                text: data.id, 
                                font: '10pt sans-serif',
                                fillColor: Cesium.Color.WHITE,
                                outlineColor: Cesium.Color.BLACK,
                                outlineWidth: 2,
                                verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
                                pixelOffset: new Cesium.Cartesian2(0, -20) 
                            }
                        });
                    } else {
                        // Actualizar posición y rotación en tiempo real
                        // vehicles[data.id].position = position;
                        const position = Cesium.Cartesian3.fromDegrees(data.longitude, data.latitude);
                        window.viewer.entities.getById(data.id).position = position;
                        window.viewer.entities.getById(data.id).orientation = Cesium.Transforms.headingPitchRollQuaternion(
                            position,
                            new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(data.angle), 0, 0)
                        );
                    }
                }


                if (data.stats){
                    document.getElementById('simulationStats').innerHTML = "Vehículos totales: " + data.stats.vehiculos_totales + "<br>" + "Tiempo medio de espera: " + data.stats.tiempo_medio_espera + "<br>" + "Emisiones de CO2 actuales: " + data.stats.emisiones_co2_actuales;
                    return;
                }

                if (data.final_report){
                    document.getElementById('final_report').innerHTML = "Total CO2: " + data.final_report.total_co2_kg + "<br>" + "Equivalente en árboles: " + data.final_report.equivalent_trees_day;
                    return;
                }   

                if (data.emisiones_xml){
                    // Aquí procesaremos el XML
                    console.log("XML de emisiones recibido");
                    const parser = new DOMParser();
                    const xmlDoc = parser.parseFromString(data.emisiones_xml, "text/xml");
                    const edges = xmlDoc.getElementsByTagName("edge");
                    let html = "";
                    for (let i = 0; i < edges.length; i++) {
                        const edge = edges[i];
                        const id = edge.getAttribute("id");
                        constCO2 = edge.getAttribute("CO2");
                        html += "<p>" + id + ": " + CO2 + "</p>";
                    }
                    document.getElementById("emisionesPorCalle").innerHTML = html;
                    return;
                }
            }
        }

        function testConnectionProxy(){
            // Antes: const socket = new WebSocket('ws://localhost:8001/ws/status');
            // Ahora (vía Nginx):
            socket = new WebSocket(window.endPoint + 'ws/status');
            // socket = new WebSocket('ws://localhost:8001/ws/status');
            document.getElementById('messages-websocket').innerHTML += "Conectando al servidor...<br/>";
            

            socket.onopen = function(e) {
                console.log("[open] Conexión establecida a través de Nginx");
                document.getElementById('messages-websocket').innerHTML += "Conexión establecida a través de Nginx<br/>";
            };

            socket.onmessage = function(event) {
                console.log(`[message] Datos recibidos: ${event.data}`);
                document.getElementById('messages-websocket').innerHTML += "Datos recibidos: " + event.data + "<br/>";
            };
        }
        