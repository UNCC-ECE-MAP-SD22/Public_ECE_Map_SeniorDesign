#Auther: Yinfei Li
#Date: OCT 8, 2021

import overpy
import pandas as pd
import os

OverpassAPI = overpy.Overpass()
if os.path.isfile('Map_data.csv') :
    df = pd.read_csv ('Map_data.csv')
else :
    df = pd.DataFrame(columns=[ 'ID', 'Longtitude', 'Latitude'])

#Class for using OSM Overpass API
class OverPass_API_Search:
    #----------------------------------------------------------
    # Search for a way by ID in OSM
    # Input parameter is str variable in Query search format
    # Query example "way(40107017); out;" 40107017 is way id
    #----------------------------------------------------------
    def QueryGetWaybyID(str):               
        result = OverpassAPI.query(str)
        way = result.ways[0]
        node = way.get_nodes(resolve_missing = True)
        return node
    #---------------------------------------------------------------
    # Search for all node way or relationship within a box in OSM
    # Query example "[out:xml]; node(50.745, 7.17, 50.75, 7.18); out;"
    # in the box parameter "node(south, west, north, est)"
    # node could change to way, relationship, nw, nwr
    #---------------------------------------------------------------
    def QueryGetInfobyBox(str):
        result = OverpassAPI.query(str)
        node = result.nodes
        way = result.ways
        return node, way


if __name__ == "__main__":
    #a = "[out:xml];nw(35.30716, -80.74561, 35.31319, -80.73666);out;"
    #node, way =  OverPass_API_Search.QueryGetInfobyBox(a)
    #for i in range (len(node)):
        #df = df.append({'ID': node[i].id, 'Longtitude' : node[i].lon, 'Latitude' : node[i].lat}, ignore_index=True)
    #df.to_csv('Map_data.csv', index = False, header = True)
    print(df)
    if 172358822 in df['ID'].values:
        print("ture")
    else :
        print("False")
