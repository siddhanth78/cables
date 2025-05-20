import folium
import geopandas as gpd
import osmnx as osx

shape_file = "mygeodata/SaiyanOfc.shp"

gdf = gpd.read_file(shape_file)

m = folium.Map()

print(gdf["seg_length"])

#folium.GeoJson(gdf).add_to(m)

m.save("test.html")
