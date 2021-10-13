#Auther: Yinfei Li
#Date: OCT 8, 2021

import overpy
import pandas as pd

OverpassAPI = overpy.Overpass()
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
    a = "[out:xml];node(35.30716, -80.74561, 35.31319, -80.73666);out;"
    node, way =  OverPass_API_Search.QueryGetInfobyBox(a)
    for i in range (len(node)):
        df = df.append({'ID': node[i].id, 'Longtitude' : node[i].lon, 'Latitude' : node[i].lat}, ignore_index=True)
    df.to_csv('export_dataframe.csv', index = False, header = True)
