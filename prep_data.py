import pandas as pd
import math
from collections import defaultdict
import json

def haversine(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371  # Radius of earth in kilometers
    
    return c * r

def prep_data(df):
    gps = df[1:]
    gps_dist = defaultdict(list)
    block_dist = {}
    code_map = {}

    for i in range(1, len(df)):
        block_dist[gps["LGD_CODE"][i]] = (gps["LGD_CODE"][i], round(haversine(gps["LAT"][i], gps["LONG"][i], df["LAT"][0], df["LONG"][0]), 2))
        code_map[gps["LGD_CODE"][i]] = (gps["GP_name"][i], gps["LAT"][i], gps["LONG"][i])
        for j in range(1, len(gps)):
            if i != j:
                dist = haversine(gps["LAT"][i], gps["LONG"][i], gps["LAT"][j], gps["LONG"][j])
                if dist <= 8.5:
                    gps_dist[gps["LGD_CODE"][i]].append((gps["LGD_CODE"][j], round(dist, 2)))

    with open("gps_dist.json", "w") as fileg:
        fileg.write(json.dumps(gps_dist, indent=4))
        
    with open("block_dist.json", "w") as fileb:
        fileb.write(json.dumps(block_dist, indent=4))

    with open("code_map.json", "w") as filec:
        filec.write(json.dumps(code_map, indent=4))

df = pd.read_csv("janikhurd.csv")
df["LGD_CODE"] = df["LGD_CODE"].astype(str)
prep_data(df)
