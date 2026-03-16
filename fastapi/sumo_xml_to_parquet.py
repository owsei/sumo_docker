import xml.etree.ElementTree as ET
import pandas as pd


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