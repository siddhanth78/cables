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
    df_dist = defaultdict(list)
    code_map = {}

    for i in range(0, len(df)):
        code_map[df["LGD_CODE"][i]] = (df["GP_name"][i], df["LAT"][i], df["LONG"][i])
        for j in range(0, len(df)):
            dist = haversine(df["LAT"][i], df["LONG"][i], df["LAT"][j], df["LONG"][j])
            df_dist[df["LGD_CODE"][i]].append(round(dist, 2))


    with open("df_dist.json", "w") as fileg:
        fileg.write(json.dumps(df_dist, indent=4))
        
    with open("code_map.json", "w") as filec:
        filec.write(json.dumps(code_map, indent=4))

df = pd.read_csv("janikhurd.csv")
df["LGD_CODE"] = df["LGD_CODE"].astype(str)
prep_data(df)
