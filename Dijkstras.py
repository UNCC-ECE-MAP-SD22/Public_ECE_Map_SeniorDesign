import math
from heapq import heapify, heappush, heappop # This is for the min heap library
import numpy as np
import sys
import random
import time
from matplotlib import pyplot as plt

numNodes = 11
edgeMatrix = np.zeros(shape = [numNodes, numNodes], dtype = int)
weightMatrix = np.zeros(shape = [numNodes, numNodes], dtype = float)
nodes = []
nodeCounter = 0
nodeCost = []
explored = [False] * numNodes
openList = []

bigNodes = [0, 2, 3, 5, 6, 7, 9, 10]
bigEdgeMatrix = np.zeros(shape = [len(bigNodes), len(bigNodes)], dtype = int)
bigWeightMatrix = np.zeros(shape = [len(bigNodes), len(bigNodes)], dtype = float)

class Node(): # defining nodes
    nodeID = -1
    xCoord = 0
    yCoord = 0
    prevNode = None
    isBuilding = False
    
    def __init__ (self, xCoord, yCoord):
        global nodeCounter
        self.xCoord = xCoord
        self.yCoord = yCoord
        self.nodeID = nodeCounter
        nodeCounter += 1
    
    def __lt__(self, other):
        global nodeCost
        if nodeCost[self.nodeID] < nodeCost[other.nodeID]:
            return True
        else:
            return False
        
    def setPrevNode(self, node):
        self.prevNode = node
    
    def setAsBuilding(self):
        self.isBuilding = True

def setup():
    global nodes
    global nodeCost
    global numNodes
    global openList
    
    heapify(openList)
    
    for i in range(numNodes):
        nodeCost.append(sys.maxsize)
    
    nodes.append(Node(0, 0))
    nodes.append(Node(.5, 1))
    nodes.append(Node(1, 1.5))
    nodes.append(Node(1.25, 2))
    nodes.append(Node(1.75, 1.75))
    nodes.append(Node(2, 2))
    nodes.append(Node(3, 2))
    nodes.append(Node(3, 2.5))
    nodes.append(Node(2, 3))
    nodes.append(Node(3, 3))
    nodes.append(Node(3, 3.5))
    
    for i in range(numNodes):
        connectNodes(i, i)
    
    connectNodes(0, 1)
    connectNodes(1, 2)
    connectNodes(2, 3)
    connectNodes(2, 4)
    connectNodes(4, 5)
    connectNodes(5, 6)
    connectNodes(5, 8)
    connectNodes(6, 7)
    connectNodes(8, 9)
    connectNodes(9, 10)
    
    calcWeights()
    
    nodes[0].setAsBuilding()
    nodes[3].setAsBuilding()
    nodes[7].setAsBuilding()
    nodes[10].setAsBuilding()
    
    global bigEdgeMatrix
    global bigWeightMatrix
    
    connectBigNodes(0, 0)
    connectBigNodes(0, 2)
    connectBigNodes(2, 2)
    connectBigNodes(2, 3)
    connectBigNodes(2, 5)
    connectBigNodes(3, 3)
    connectBigNodes(5, 5)
    connectBigNodes(5, 6)
    connectBigNodes(5, 9)
    connectBigNodes(6, 6)
    connectBigNodes(6, 7)
    connectBigNodes(7, 7)
    connectBigNodes(9, 9)
    connectBigNodes(9, 10)
    connectBigNodes(10, 10)
    
    for i in range(len(bigNodes)):
        for j in range(len(bigNodes)):
            if bigEdgeMatrix[i, j] == 1:
                cost = edgeCost(nodes[bigNodes[i]], nodes[bigNodes[j]])
                bigWeightMatrix[i, j] = cost

def edgeCost(node1, node2): # defining the edges using the distance formula
    xdist = node1.xCoord - node2.xCoord
    ydist = node1.yCoord - node2.yCoord
    dist = math.sqrt(pow(xdist, 2) + pow(ydist, 2))
    return dist

def connectNodes(node1, node2):
    global edgeMatrix
    edgeMatrix[node1, node2] = 1
    edgeMatrix[node2, node1] = 1

def connectBigNodes(node1, node2):
    global bigEdgeMatrix
    global bigNodes
    bigEdgeMatrix[bigNodes.index(node1), bigNodes.index(node2)] = 1
    bigEdgeMatrix[bigNodes.index(node2), bigNodes.index(node1)] = 1

def calcWeights():
    global edgeMatrix
    global weightMatrix
    global nodes
    for i in range(numNodes):
        for j in range(numNodes):
            if edgeMatrix[i, j] == 1:
                cost = edgeCost(nodes[i], nodes[j])
                weightMatrix[i, j] = cost

def setStart(node):
    global nodeCost
    global openList
    
    nodeCost[node.nodeID] = 0
    explored[node.nodeID] = True
    heappush(openList, node)

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

    for i in range(numNodes):
        plt.plot(nodes[i].xCoord, nodes[i].yCoord, marker = "o")

    for i in range(numNodes):
        for j in range (numNodes):
            if edgeMatrix[i][j] != 0 :
                plt.plot([nodes[i].xCoord, nodes[j].xCoord],[nodes[i].yCoord, nodes[j].yCoord], 'b')

    currentNode = node
    prevNode = node.prevNode

    while prevNode != None:
        plt.plot([currentNode.xCoord, prevNode.xCoord], [currentNode.yCoord, prevNode.yCoord], 'r')
        currentNode = prevNode
        prevNode = currentNode.prevNode
    plt.show()
    

def bigExplore(node):
    global bigWeightMatrix
    global nodeCost
    global bigNodes
    global nodes
    global openList
    
    currentNode = node.nodeID
    
    for nextNode in range(len(bigNodes)):
        if not explored[bigNodes[nextNode]]:
            if bigWeightMatrix[bigNodes.index(currentNode)][nextNode] != 0:
                newCost = bigWeightMatrix[bigNodes.index(currentNode)][nextNode] + nodeCost[currentNode]
                
                if newCost < nodeCost[bigNodes[nextNode]]:
                    nodeCost[bigNodes[nextNode]] = newCost
                    nodes[bigNodes[nextNode]].setPrevNode(node)
                    heappush(openList, nodes[bigNodes[nextNode]])
    
def findBigPath(startNode, destination):
    global bigNodes
    global bigWeightMatrix
    global nodeCost
    global openList
    global explored
    global nodes
    
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
        bigExplore(node)
        explored[node.nodeID] = True
        node = heappop(openList)
    
    #reset the algorithm
    while len(openList) != 0:
        heappop(openList)
    
    explored = [False] * numNodes
    
    return node

setup()

findPath(nodes[0], nodes[10])
findPath(nodes[0], nodes[7])
node = findBigPath(nodes[0], nodes[10])

for i in range(numNodes):
    plt.plot(nodes[i].xCoord, nodes[i].yCoord, marker = "o")

for i in range(numNodes):
        for j in range (numNodes):
            if edgeMatrix[i][j] != 0 :
                plt.plot([nodes[i].xCoord, nodes[j].xCoord],[nodes[i].yCoord, nodes[j].yCoord], 'b')

currentNode = node
prevNode = currentNode.prevNode                
while prevNode != None:
    plt.plot([currentNode.xCoord, prevNode.xCoord], [currentNode.yCoord, prevNode.yCoord], 'r')
    currentNode = prevNode
    prevNode = currentNode.prevNode
plt.show()
