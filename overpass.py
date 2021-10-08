from numpy import matrix, number
import overpy
import pandas as pd

api = overpy.Overpass()
result = api.query("way(40107017); out;")
way = result.ways[0]
node = way.get_nodes(resolve_missing=True)

df = pd.DataFrame(columns=[ 'ID', 'Longtitude', 'Latitude','Connection1', 'Connection2'])
for i in range (len(node)):
    df = df.append({'ID': node[i].id, 'Longtitude' : node[i].lon, 'Latitude' : node[i].lat}, ignore_index=True)
    if i == 0:
        df.at[i, 'Connection1'] = node[i+1].id
    elif i == ((len(node))-1):
        df.at[i, 'Connection1'] = node[i-1].id
    else :
        df.at[i, 'Connection1'] = node[i-1].id
        df.at[i, 'Connection2'] = node[i+1].id
df.to_csv('export_dataframe.csv', index = False, header = True)
