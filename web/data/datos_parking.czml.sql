WITH temporal_data AS (
    -- 1. Extraemos y calculamos los colores por cada fila (Verde a Rojo)
    SELECT 
        entityid,
        name,
        ST_X(location) AS lon, 
		ST_Y(location) AS lat,
        TO_CHAR(recvtime, 'YYYY-MM-DD HH24:MI:SS') as ts,
        (percentageoccupancy * 2.55)::int as r,
        ((100 - percentageoccupancy) * 2.55)::int as g
    FROM sc_pamplona_pro.parking_offstreetparking
    WHERE recvtime BETWEEN '2026-03-01 00:00:00' AND '2026-03-01 23:59:59' -- Ajusta tu rango
    ORDER BY recvtime desc
),
flattened_rgba AS (
    -- 2. Aplanamos los datos para que el array sea [t1,r1,g1,b1,a1, t2,r2,g2,b2,a2...]
    -- Esto es vital para que Cesium entienda la línea de tiempo correctamente
    SELECT 
        entityid, 
        name, 
        lon, 
		lat,
        jsonb_agg(val) as rgba_array
    FROM (
        SELECT 
            entityid, name, lon, lat,
            unnest(array[to_jsonb(ts::text), to_jsonb(r::text), to_jsonb(g::text), '0'::jsonb, '255'::jsonb]) as val
        FROM temporal_data
    ) sub
    GROUP BY entityid, name, lon, lat
),
parking_packets AS (
    -- 3. Construimos el paquete de cada entidad (el parking)
    SELECT jsonb_build_object(
        'id', 'parking_' || entityid,
        'name', name,
        'position', jsonb_build_object('cartographicDegrees', jsonb_build_array(lon, lat, 0)),
        'ellipse', jsonb_build_object(
            'semiMajorAxis', 25,
            'semiMinorAxis', 25,
            'height', 0,
            'extrudedHeight', 12,
            'material', jsonb_build_object(
                'solidColor', jsonb_build_object(
                    'color', jsonb_build_object('rgba', rgba_array)
                )
            )
        )
    ) as packet
    FROM flattened_rgba
),
doc_header AS (
    -- 4. El paquete 'document' con la configuración del reloj de Cesium
    SELECT jsonb_build_object(
        'id', 'document',
        'version', '1.0',
        'clock', jsonb_build_object(
            'interval', (SELECT min(ts) FROM temporal_data) || '/' || (SELECT max(ts) FROM temporal_data),
            'currentTime', (SELECT min(ts) FROM temporal_data),
            'multiplier', 60,
            'range', 'LOOP_STOP',
            'step', 'SYSTEM_CLOCK_MULTIPLIER'
        )
    ) as header
)
-- 5. Unión final en un único array CZML
SELECT jsonb_pretty(jsonb_agg(data))
FROM (
    SELECT header FROM doc_header
    UNION ALL
    SELECT packet FROM parking_packets
) as final_czml(data);