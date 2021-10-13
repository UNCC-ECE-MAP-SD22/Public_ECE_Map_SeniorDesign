#9/9/21
import math
from heapq import heapify, heappush, heappop # This is for the min heap library
import numpy as np

numNodes = 4
edgeMatrix = np.zeros(shape = [numNodes, numNodes], dtype = float)
weightMatrix = np.zeros(shape = [numNodes, numNodes], dtype = float)
nodeCounter = 0

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

class Node(): # defining nodes
    nodeID = -1
    xCoord = 0
    yCoord = 0
    cost = -1
    prevNode = None
    
    def __init__ (self, xCoord, yCoord): # so you can pass values to a new node
        global nodeCounter
        self.xCoord = xCoord
        self.yCoord = yCoord
        self.cost = 99999999999.9
        self.nodeID = nodeCounter
        nodeCounter += 1

    def updateCost(self, prevNode):
        if self.cost == 0:
            return
        global weightMatrix
        edge = weightMatrix[self.nodeID, prevNode.nodeID] + prevNode.cost
        
        if(edge < self.cost):
            self.cost = edge
            self.prevNode = prevNode
    
    def setStart(self):
        self.cost = 0
        global numNodes
        global nodes
        for i in range(numNodes):
            if edgeMatrix[self.nodeID, nodes[i].nodeID] == 1:
                nodes[i].updateCost(self)
    
    def explore(self):
        global numNodes
        global nodes
        for i in range(numNodes):
            if edgeMatrix[self.nodeID, nodes[i].nodeID] == 1:
                nodes[i].updateCost(self)


nodes = []
nodes.append(Node(0,0))
nodes.append(Node(3,4))
nodes.append(Node(2,1))
nodes.append(Node(7,8))

connectNodes(0,1)
connectNodes(0,2)
connectNodes(1,2)
connectNodes(1,3)
connectNodes(2,3)

calcWeights()

nodes[0].setStart()
destination = nodes[3]

openList = []
heapify(openList)
for node in nodes:
    heappush(openList, (node.cost, node))

heappop(openList)

while openList[0][1] != destination:
    openList[0][1].explore()
    heappop(openList)
    
    if openList[0][1] == destination:
        print(openList[0][1].nodeID)

print(openList[0][1].prevNode.nodeID)
print(openList[0][1].prevNode.prevNode.nodeID)

