#Auther: Yinfei Li
#Co-Worker:
#Date: OCT 8, 2021

import overpy
import pandas as pd
import time

OverpassAPI = overpy.Overpass()

#Class for using OSM Overpass API
class OverPass_API_Search:
    #/---------------------------------------------------------
    # Search for a way (ROAD) in OSM
    # Input parameter is str variable in Query search format
    # Query example "way(40107017); out;" 40107017 is way id
    #/----------------------------------------------------------
    def QueryGetWay(str):               
        result = OverpassAPI.query(str)
        way = result.ways[0]
        node = way.get_nodes(resolve_missing = True)
        return node

if __name__ == "__main__":
    #time1 = time.time()
    df = pd.DataFrame(columns=[ 'ID', 'Longtitude', 'Latitude'])
    node = OverPass_API_Search.QueryGetWay("way(40107017); out;")
    node2 = OverPass_API_Search.QueryGetWay("way(); out;")
    #time2 = time.time()
    for i in range (len(node)):
        df = df.append({'ID': node[i].id, 'Longtitude' : node[i].lon, 'Latitude' : node[i].lat}, ignore_index=True)
    #time3 = time.time()
    #i = int(df[df['ID'] == 482708531].index.values)
    #print(i)
    #for i in range (len(node)):
        #print(df[df['ID'] == node[i].id])
    #print(type(df.iloc[0]['ID']))
    #time4 = time.time()
    #print(time2-time1)
    #print(time3-time2)
    #print(time4-time3)
    
