from numpy import number
import overpy
import pandas as pd

api = overpy.Overpass()
result = api.query("way(40107019); out;")
way = result.ways[0]
node = way.get_nodes(resolve_missing=True)


#node = result.nodes[0]
#print(node.id)
#print(node.lat)
#print(node.lon)

df = pd.DataFrame(columns=[ 'ID', 'Longtitude', 'Latitude'])
for i in range (len(node)):
    df = df.append({'ID': node[i].id, 'Longtitude' : node[i].lon, 'Latitude' : node[i].lat}, ignore_index=True)
print(df)