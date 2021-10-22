# In[1]:


# Imports
import math
from heapq import heapify, heappush, heappop # This is for the min heap library
import numpy as np # probably not necessary, but is used for initializing the edge and weights
import sys # used for setting the costs to inf (could be replaced with just using a really large number)
import time # used to get the runtime of the algorithm
from matplotlib import pyplot as plt # used to graph the path taken


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
    path = []
    
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
    
    # set as building to be shown on the map
    def setBuilding(self):
        self.isBuilding = True
    
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
    
    heapify(openList)
    
    # set costs to max
    for i in range(numNodes):
        nodeCost.append(sys.maxsize)
    
    epic = Node(buildings["epic"][0], buildings["epic"][1])
    epic.setBuilding()
    
    epic_duke = makeNode("gpx/epic_duke.gpx")
    
    duke = Node(buildings["duke"][0], buildings["duke"][1])
    duke.setBuilding()
    
    duke_bio = makeNode("gpx/duke_bio.gpx")
    
    bio = Node(buildings["bio"][0], buildings["bio"][1])
    bio.setBuilding()
    
    bio_grigg = makeNode("gpx/bio_grigg.gpx")
    
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

# defining the edges using the distance formula
def edgeCost(node1, node2): 
    xdist = node1.xCoord - node2.xCoord
    ydist = node1.yCoord - node2.yCoord
    dist = math.sqrt(pow(xdist, 2) + pow(ydist, 2))
    return dist

# symmetrically connenct 2 nodes
def connectNodes(node1, node2):
    global edgeMatrix
    edgeMatrix[node1, node2] = 1
    edgeMatrix[node2, node1] = 1

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
    
    # reset the algorithm
    while len(openList) != 0:
        heappop(openList)
    
    # set costs to max, remove all previous node data
    for i in range(numNodes):
        nodeCost[i] = sys.maxsize
        nodes[i].setPrevNode(None)
    
    explored = [False] * numNodes
    
    # set start node
    setStart(startNode)
    
    node = heappop(openList)
    
    # eplore until we find the destination
    while node != destination:
        explore(node)
        explored[node.nodeID] = True
        node = heappop(openList)
    
    path = []
    dest = [node.xCoord, node.yCoord]
    
    # walk backwards through our explored nodes and extract the path taken
    while node != None:
        if node.isBuilding:
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
                
                #print(dest)
                #print(roadPath[1], roadPath[0])
                #print(roadPath[-1], roadPath[-2])
                
        # continue working backwards
        node = node.prevNode
    
    # reverse the entire path to get the path from start to destination
    path.reverse()
    return path


# In[8]:


setup()

print("Start", time.time())

# run the algorithm between 2 points, returning the path to graph
path = findPath(nodes[0], nodes[numNodes-1])
#path = findPath(nodes[numNodes-1], nodes[0])

print("Finish", time.time())


# In[9]:


# plot the full path taken in an isometric view
for i in range(numNodes):
    if nodes[i].isBuilding:
        plt.plot(nodes[i].yCoord, nodes[i].xCoord, marker = "o")
i = 0
while (i + 3) < len(path):
    plt.plot([path[i + 1], path[i + 3]], [path[i + 0], path[i + 2]], 'r')
    i += 2
plt.show()


# In[10]:


# plot the full path taken in a top down view
for i in range(numNodes):
    if nodes[i].isBuilding:
        plt.plot(nodes[i].xCoord, -nodes[i].yCoord, marker = "o")
i = 0
while (i + 3) < len(path):
    plt.plot([path[i + 0], path[i + 2]], [-path[i + 1], -path[i + 3]], 'r')
    i += 2
plt.show()
