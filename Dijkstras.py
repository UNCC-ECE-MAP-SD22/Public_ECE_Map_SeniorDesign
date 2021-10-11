# In[1]:


# Imports
import math
from heapq import heapify, heappush, heappop # This is for the min heap library
import numpy as np
import sys
#import random 
#import time
from matplotlib import pyplot as plt


# In[2]:


# Set up variables for dijkstra's
numNodes = 7
edgeMatrix = np.zeros(shape = [numNodes, numNodes], dtype = int)
weightMatrix = np.zeros(shape = [numNodes, numNodes], dtype = float)
nodes = []
nodeCounter = 0
nodeCost = []
explored = [False] * numNodes
openList = []

# defines coordinates of each building
buildings = {
    "epic": [35.30923, -80.74114],
    "duke": [35.31222, -80.74083],
    "bio": [35.31289, -80.74168],
    "grigg": [35.31143, -80.7427],
}


# In[3]:


# Node class
class Node(): # defining nodes
    nodeID = -1
    xCoord = 0
    yCoord = 0
    prevNode = None
    isBuilding = False
    forwardPath = []
    backPath = []
    
    # initialize an instance
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
    
    def setBuilding(self):
        self.isBuilding = True
    
    # used for the path storage in the abstraction
    def setPath(self, path):
        self.forwardPath = path
        temp = path
        temp.reverse()
        self.backPath = temp
    
    # return the path stored in the big node (the coordinates along the raod section)
    def getPath(self, startCoord):
        if len(self.forwardPath) == 0:
            return
        
        if startCoord == [self.forwardPath[0], self.forwardPath[1]]:
            return self.forwardPath
        return self.backPath


# In[4]:


def makeNode(filename):
    #parse GPX data
    route = open(filename, 'r')
    route = route.read()

    center_x = 0.0
    center_y = 0.0
    road_path = []

    for i in range(len(route)):
        if i >= (len(route) - 6):
            break

        if (route[i] + route[i+1] + route[i+2] + route[i+3] + route[i+4] + route[i+5]) == "center":
            j = i+7
            lat = ""
            while route[j] != '%':
                lat += route[j]
                j += 1

            j += 3
            lon = ""
            while route[j] != '&':
                lon += route[j]
                j += 1
            center_x = float(lat)
            center_y = float(lon)


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
    streetNode = Node(center_x, center_y)
    streetNode.setPath(road_path)
    
    return streetNode


# In[5]:


# initial setup
def setup():
    global nodes
    global nodeCost
    global numNodes
    global openList
    
    heapify(openList)
    
    # set costs to max
    for i in range(numNodes):
        nodeCost.append(sys.maxsize)
    
    epic = Node(buildings["epic"][0], buildings["epic"][1])
    epic.setBuilding()
    
    epic_duke = makeNode("epic_duke.gpx")
    
    duke = Node(buildings["duke"][0], buildings["duke"][1])
    duke.setBuilding()
    
    duke_bio = makeNode("duke_bio.gpx")
    
    bio = Node(buildings["bio"][0], buildings["bio"][1])
    bio.setBuilding()
    
    bio_grigg = makeNode("bio_grigg.gpx")
    
    grigg = Node(buildings["grigg"][0], buildings["grigg"][1])
    grigg.setBuilding()
    
    nodes.append(epic)
    nodes.append(epic_duke)
    nodes.append(duke)
    nodes.append(duke_bio)
    nodes.append(bio)
    nodes.append(bio_grigg)
    nodes.append(grigg)
    
    for i in range(numNodes):
        connectNodes(i, i)
        if(i < numNodes-1):
            connectNodes(i, i + 1)
    
    calcWeights()


# In[6]:


# set up functions used in setup
def edgeCost(node1, node2): # defining the edges using the distance formula
    xdist = node1.xCoord - node2.xCoord
    ydist = node1.yCoord - node2.yCoord
    dist = math.sqrt(pow(xdist, 2) + pow(ydist, 2))
    return dist

def connectNodes(node1, node2):
    global edgeMatrix
    edgeMatrix[node1, node2] = 1
    edgeMatrix[node2, node1] = 1

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
    
    #reset the algorithm
    while len(openList) != 0:
        heappop(openList)
    
    for i in range(numNodes):
        nodeCost[i] = sys.maxsize
        nodes[i].setPrevNode(None)
    
    explored = [False] * numNodes
    
    #set start node
    setStart(startNode)
    
    node = heappop(openList)

    while node != destination:
        explore(node)
        explored[node.nodeID] = True
        node = heappop(openList)
    
    path = []
    d = buildings["grigg"]
    while node != None:
        roadPath = node.getPath(d)
        if roadPath != None:
            for i in roadPath:
                path.append(i)
        node = node.prevNode
    
    path.reverse()
    return path


# In[8]:


# run the algorithm between 2 points
setup()

path = findPath(nodes[0], nodes[numNodes-1])


# In[9]:


# plot the full path taken
for i in range(numNodes):
        if nodes[i].isBuilding:
            plt.plot(nodes[i].yCoord, nodes[i].xCoord, marker = "o")
i = 0
while (i + 3) < len(path):
    plt.plot([path[i + 1], path[i + 3]], [path[i + 0], path[i + 2]], 'r')
    i += 2
plt.show()


