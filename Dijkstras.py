#!/usr/bin/env python
# coding: utf-8

# In[1]:


# Imports
import math
from heapq import heapify, heappush, heappop # This is for the min heap
import numpy as np # probably not necessary, but is used for initializing the edge and weights
import sys # used for setting the costs to inf (could be replaced with just using a really large number)
import time # used to get the runtime of the algorithm
from matplotlib import pyplot as plt # used to graph the path taken


# In[2]:


# Set up variables for dijkstra's
numNodes = 23
edgeMatrix = np.zeros(shape = [numNodes, numNodes], dtype = int)
weightMatrix = np.zeros(shape = [numNodes, numNodes], dtype = float)
nodes = []
nodeCounter = 0
nodeCost = []
explored = [False] * numNodes
openList = []

# defines coordinates of each big node (buildings and intersections)
bigNodes = {
    "phillips": [35.30888, -80.74064],
    "epic": [35.30934, -80.74132],
    "grigg": [35.31092, -80.74148],
    "duke": [35.31163, -80.74076],
    "motorsports": [35.3124, -80.74048],
    "robSny": [35.31287, -80.74115],
    "bioinformatics": [35.31233, -80.74176],
    "circBio": [35.31211, -80.74199],
    "circInst": [35.31207, -80.74233],
    "circPortal": [35.31178, -80.74235],
    "portal": [35.31147, -80.74266],
}


# In[3]:


# Node class
class Node(): # defining nodes
    nodeID = -1
    xCoord = 0
    yCoord = 0
    prevNode = None
    isBigNode = False
    path = []
    
    # instantiate a node with given GPS coords
    def __init__ (self, xCoord, yCoord):
        global nodeCounter
        self.xCoord = xCoord
        self.yCoord = yCoord
        self.nodeID = nodeCounter
        nodeCounter += 1
    
    # used for less than comparisons (min heap)
    def __lt__(self, other):
        global nodeCost
        if nodeCost[self.nodeID] < nodeCost[other.nodeID]:
            return True
        else:
            return False
        
    # keep track of what node you came from
    def setPrevNode(self, node):
        self.prevNode = node
    
    # set as big node to be shown on the map
    def setBigNode(self):
        self.isBigNode = True
    
    # used for the path storage in the abstraction
    def setPath(self, newPath):
        newPath.reverse()
        self.path = newPath
    
    # return the path stored in the big node (the coordinates along the raod section)
    def getPath(self, startCoord):
        if len(self.path) == 0:
            return
        
        return self.path


# In[4]:


#parse GPX data from GPX files
def makeNode(filename): 
    route = open(filename, 'r')
    route = route.read()

    road_path = []
    atCoords = False
    
    # iterate through the GPX file and grab the latitudes and longitudes
    for i in range(len(route)):
        if i >= (len(route) - 4):
            break
        if (route[i] + route[i+1] + route[i+2] + route[i+3] + route[i+4]) == "<trk>":
            atCoords = True
        if atCoords:
            if (route[i] + route[i+1] + route[i+2]) == "lat":
                j = i+5
                lat = ""
                while route[j] != '"':
                    lat += route[j]
                    j += 1
                road_path.append(float(lat))

            if (route[i] + route[i+1] + route[i+2]) == "lon":
                j = i+5
                lon = ""
                while route[j] != '"':
                    lon += route[j]
                    j += 1
                road_path.append(float(lon))
                
    # create an abstracted location for Dijkstra's using the average of the GPS coords
    xSum = 0
    ySum = 0
    for i in range(len(road_path)):
        if i%2:
            ySum += road_path[i]
        else:
            xSum += road_path[i]
    
    center_x = xSum / (len(road_path) / 2)
    center_y = ySum / (len(road_path) / 2)
    
    # create and instantiate the node class
    streetNode = Node(center_x, center_y)
    streetNode.setPath(road_path)
    
    # return the node
    return streetNode


# In[5]:


# initial setup
def setup():
    global nodes
    global nodeCost
    global numNodes
    global openList
    
    # convert to heap
    heapify(openList)
    
    # set costs to max
    for i in range(numNodes):
        nodeCost.append(sys.maxsize)
    
    # creates building, intersections, and path nodes
    
    wellsFargo_phillipX =makeNode("gpx/phillips_wellsfargo_phillips_phillips.gpx")
    
    phillipsX = Node(bigNodes["phillips"][0], bigNodes["phillips"][1])
    phillipsX.setBigNode()
    
    phillipX_epic = makeNode("gpx/phillips_phillips_In-front-of-Epic.gpx")
    
    epic = Node(bigNodes["epic"][0], bigNodes["epic"][1])
    epic.setBigNode()
    
    epic_grigg = makeNode("gpx/In-front-of-Epic_In-front-of-Grigg.gpx")
    
    grigg = Node(bigNodes["grigg"][0], bigNodes["grigg"][1])
    grigg.setBigNode()
    
    grigg_duke = makeNode("gpx/In-Front-of-Grigg_In-Front-of-Duke.gpx")
    
    duke = Node(bigNodes["duke"][0], bigNodes["duke"][1])
    duke.setBigNode()
    
    duke_motor = makeNode("gpx/In-front-of-Duke_In-front-of-Motorsports.gpx")
    
    motor = Node(bigNodes["motorsports"][0], bigNodes["motorsports"][1])
    motor.setBigNode()
    
    motor_robSnyX = makeNode("gpx/In-front-of-MSRB_robert-snyder_phillips.gpx")
    
    robSnyX = Node(bigNodes["robSny"][0], bigNodes["robSny"][1])
    robSnyX.setBigNode()
    
    robSnyX_bio = makeNode("gpx/robert-snyder_phillips_In-front-of-BioInfo.gpx")
    
    bio = Node(bigNodes["bioinformatics"][0], bigNodes["bioinformatics"][1])
    bio.setBigNode()
    
    bio_circBio = makeNode("gpx/In-front-of-BioInfo_Institutecir_Bio.gpx")
    
    circBio = Node(bigNodes["circBio"][0], bigNodes["circBio"][1])
    circBio.setBigNode()
    
    circBio_circInst = makeNode("gpx/institutecir-bio_institute-institutecir.gpx")
    
    circInst = Node(bigNodes["circInst"][0], bigNodes["circInst"][1])
    circInst.setBigNode()
    
    circInst_circPortal = makeNode("gpx/institutecir-institute_institutecir-portal.gpx")
    
    circPortal = Node(bigNodes["circPortal"][0], bigNodes["circPortal"][1])
    circPortal.setBigNode()
    
    circPortal_circBio = makeNode("gpx/Institutecir-portal_institute-BioInfo_UNDERSIDE.gpx")
    
    circPortal_portal = makeNode("gpx/institutecir-portal_In-front-of-Portal.gpx")
    
    portal = Node(bigNodes["portal"][0], bigNodes["portal"][1])
    portal.setBigNode()
    
    
    # append all nodes to the nodes list
    nodes.append(wellsFargo_phillipX) #0
    nodes.append(phillipsX)
    nodes.append(phillipX_epic)
    nodes.append(epic)
    nodes.append(epic_grigg)
    nodes.append(grigg)
    nodes.append(grigg_duke)
    nodes.append(duke)
    nodes.append(duke_motor)
    nodes.append(motor)
    nodes.append(motor_robSnyX) #10
    nodes.append(robSnyX)
    nodes.append(robSnyX_bio)
    nodes.append(bio)
    nodes.append(bio_circBio)
    nodes.append(circBio)
    nodes.append(circBio_circInst)
    nodes.append(circInst)
    nodes.append(circInst_circPortal)
    nodes.append(circPortal)
    nodes.append(circPortal_circBio) #20
    nodes.append(circPortal_portal)
    nodes.append(portal)
    
    
    # connect each node to the next node and itself
    connectNodes( 0 , 1 )
    connectNodes( 1 , 2 )
    connectNodes( 2 , 3 )
    connectNodes( 3 , 4 )
    connectNodes( 4 , 5 )
    connectNodes( 5 , 6 )
    connectNodes( 6 , 7 )
    connectNodes( 7 , 8 )
    connectNodes( 8 , 9 )
    connectNodes( 9 , 10 )
    connectNodes( 10 , 11 )
    connectNodes( 11 , 12 )
    connectNodes( 12 , 13 )
    connectNodes( 13 , 14 )
    connectNodes( 14 , 15 )
    connectNodesAsym( 15 , 16 )
    connectNodesAsym( 16 , 17 )
    connectNodesAsym( 17 , 18 )
    connectNodesAsym( 18 , 19 )
    connectNodesAsym( 19 , 20 )
    connectNodesAsym( 20 , 15 )
    connectNodes( 19 , 21 )
    connectNodes( 21 , 22 )
    
    # update the weight matrix
    calcWeights()


# In[6]:


# defining the edges using the distance formula
def edgeCost(node1, node2): 
    xdist = node1.xCoord - node2.xCoord
    ydist = node1.yCoord - node2.yCoord
    dist = math.sqrt(pow(xdist, 2) + pow(ydist, 2))
    return dist

# symmetrically connect 2 nodes
def connectNodes(node1, node2):
    global edgeMatrix
    edgeMatrix[node1, node2] = 1
    edgeMatrix[node2, node1] = 1

# asymmetrically connect 2 nodes
def connectNodesAsym(node1, node2):
    global edgeMatrix
    edgeMatrix[node1, node2] = 1
    
# update the weight matrix using the edgeCost function
def calcWeights():
    global edgeMatrix
    global weightMatrix
    global nodes
    for i in range(numNodes):
        for j in range(numNodes):
            if edgeMatrix[i, j] == 1:
                cost = edgeCost(nodes[i], nodes[j])
                weightMatrix[i, j] = cost

# set initial node
def setStart(node):
    global nodeCost
    global openList
    
    nodeCost[node.nodeID] = 0
    explored[node.nodeID] = True
    heappush(openList, node)

    
# explore a given node
def explore(node):
    global weightMatrix
    global nodeCost
    global numNodes
    global nodes
    global openList
    
    currentNode = node.nodeID
    
    for nextNode in range(numNodes):
        if not explored[nextNode]:
            if weightMatrix[currentNode][nextNode] != 0:
                newCost = weightMatrix[currentNode][nextNode] + nodeCost[currentNode]
                
                if newCost < nodeCost[nextNode]:
                    nodeCost[nextNode] = newCost
                    nodes[nextNode].setPrevNode(node)
                    heappush(openList, nodes[nextNode])


# In[7]:


# defining the algorithm to find and graph the path to two points
def findPath(startNode, destination):
    global nodes
    global weightMatrix
    global nodeCost
    global numNodes
    global openList
    global explored
    
    # clear the heap
    while len(openList) != 0:
        heappop(openList)
    
    # set costs to max, remove all previous node data
    for i in range(numNodes):
        nodeCost[i] = sys.maxsize
        nodes[i].setPrevNode(None)
    
    # set all nodes as unexplored
    explored = [False] * numNodes
    
    # set start node
    setStart(startNode)
    
    node = heappop(openList)
    
    # explore until we find the destination
    while node != destination:
        explore(node)
        explored[node.nodeID] = True
        node = heappop(openList)
    
    path = []
    dest = [node.xCoord, node.yCoord]
    
    # walk backwards through our explored nodes and extract the path taken
    while node != None:
        if node.isBigNode:
            dest = [node.xCoord, node.yCoord]
        else:
            roadPath = node.getPath(dest)
            
            if roadPath != None:
                # reverse the path if coming from the opposite direction
                if dest != [roadPath[1], roadPath[0]]:
                    i = len(roadPath) - 1
                    while (i - 1) >= 0:
                        path.append(roadPath[i - 1])
                        path.append(roadPath[i])
                        i -= 2
                
                # otherwise keep the original stored direction
                else:
                    for i in roadPath:
                        path.append(i)
                
        # continue working backwards
        node = node.prevNode
    
    # reverse the entire path to get the path from start to destination
    path.reverse()
    return path


# In[8]:


setup()

print("Start", time.time())

# run the algorithm between 2 points, returning the path to graph
path = findPath(nodes[3], nodes[numNodes-1])
#path = findPath(nodes[numNodes-1], nodes[7])
#path = findPath(nodes[7], nodes[9])

print("Finish", time.time())


# In[9]:


# plot the full path taken in an isometric view
for i in range(numNodes):
    if nodes[i].isBigNode:
        plt.plot(nodes[i].yCoord, nodes[i].xCoord, marker = "o")
i = 0
while (i + 3) < len(path):
    plt.plot([path[i + 1], path[i + 3]], [path[i + 0], path[i + 2]], 'r')
    i += 2
plt.show()


# In[10]:


# plot the full path taken in a top down view
for i in range(numNodes):
    if nodes[i].isBigNode:
        plt.plot(nodes[i].xCoord, -nodes[i].yCoord, marker = "o")
i = 0
while (i + 3) < len(path):
    plt.plot([path[i + 0], path[i + 2]], [-path[i + 1], -path[i + 3]], 'r')
    i += 2
plt.show()


# In[11]:


# convert the path into the array format of Matlab (x1 y1; x2 y2; etc.)
matlabPath = ""
for i in range(len(path)):
    if i == 0:
        matlabPath += str(path[i])
    else:
        matlabPath += " " + str(path[i])
    if i%2 == 1 and i != 0:
        matlabPath += ";"
#print(matlabPath)


# In[12]:


# post processing on the path
postPath = []

# minimum distance between nodes
D = 0.0001
i = 0
while i < len(path) - 4:
    xdist = path[i+2] - path[i]
    ydist = path[i+3] - path[i+1]
    dist = math.sqrt(pow(xdist, 2) + pow(ydist, 2))
    
    # if no repeaeted nodes, add the first node
    if xdist != 0 and ydist != 0:
        postPath.append(path[i])
        postPath.append(path[i+1])
    
    if dist >= D:
        numNewNodes = dist // D
        scaling = D / dist
        for j in range(int(numNewNodes)):
            xCoord = (j+1) * xdist/numNewNodes + path[i]
            yCoord = (j+1) * ydist/numNewNodes + path[i+1]
            
            postPath.append(xCoord)
            postPath.append(yCoord)
    i += 2
postPath.append(path[-2])
postPath.append(path[-1])

#remove any repeated nodes
i = 0
fixedPath = []

while i < len(postPath) - 4:
    xdist = postPath[i+2] - postPath[i]
    ydist = postPath[i+3] - postPath[i+1]
    if xdist != 0 and ydist != 0:
        fixedPath.append(postPath[i])
        fixedPath.append(postPath[i+1])
    i += 2
fixedPath.append(postPath[-4])
fixedPath.append(postPath[-3])
fixedPath.append(postPath[-2])
fixedPath.append(postPath[-1])

#path = fixedPath


# In[13]:


i = 0
while i < len(path) - 1:
    plt.plot(path[i], path[i+1], marker = "o")
    i += 2


# In[14]:


i = 0
while i < len(postPath) - 1:
    plt.plot(postPath[i], postPath[i+1], marker = "o")
    i += 2


# In[15]:


turnPath = []

temp = path
path = fixedPath

i = 0
beenCurved = False
while i < len(path) - 6:
    xDelta1 = path[i+2] - path[i]# + .000001
    yDelta1 = path[i+3] - path[i+1]# + .000001
    
    xDelta2 = path[i+4] - path[i+2]# + .000001
    yDelta2 = path[i+5] - path[i+3]# + .000001
    
    m1 = yDelta1 / xDelta2
    m2 = yDelta2 / xDelta2
    
    theta = math.atan((m2-m1) / (1+m1*m2))
    #print(theta)
    
    # set 15 degree max turn
    turn = 3.14 * 15/180
    # set distance between nodes
    H = 0.00005
    
    if beenCurved:
        beenCurved = False
    elif abs(theta) >= turn:
        beenCurved = True
        
        def getValue(x1,x2,t):
            return x1+(x2-x1)*t

        def draw(x,y):
            x0 = getValue(x[0], x[1], j / N)
            y0 = getValue(y[0], y[1], j / N)
            x1 = getValue(x[1], x[2], j / N)
            y1 = getValue(y[1], y[2], j / N)
            xCoords.append(getValue(x0, x1, j / N))
            yCoords.append(getValue(y0, y1, j / N))

            return

        xCoords=[]
        yCoords=[]
        N = 5
        x = [path[i], path[i+2], path[i+4]]
        y = [path[i+1], path[i+3], path[i+5]]

        for j in range(N+1):
            draw(x,y)
        
        for k in range(len(xCoords)):
            turnPath.append(xCoords[k])
            turnPath.append(yCoords[k])
    else:
        turnPath.append(path[i])
        turnPath.append(path[i+1])
    i += 2
turnPath.append(path[-4])
turnPath.append(path[-3])
turnPath.append(path[-2])
turnPath.append(path[-1])
    
path = temp


# In[16]:


i = 0
while i < len(turnPath) - 1:
    plt.plot(turnPath[i], turnPath[i+1], marker = "o")
    i += 2


# In[ ]:




