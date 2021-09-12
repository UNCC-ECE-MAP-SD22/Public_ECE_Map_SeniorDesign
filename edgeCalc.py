#9/9/21
import math
from heapq import heapify, heappush, heappop # This is for the min heap library

def edgeCost(node1, node2): # defining the edges using the distance formula
    xdist = node1.xCoord - node2.xCoord
    ydist = node1.yCoord - node2.yCoord
    dist = math.sqrt(pow(xdist, 2) + pow(ydist, 2))
    return dist

class Node(): # defining nodes
    connectedNodes = []
    nodeID = 0
    xCoord = 0
    yCoord = 0
    cost = 0
    prevNode = None
    
    def __init__ (self, xCoord, yCoord): # so you can pss values to a new node
        self.xCoord = xCoord
        self.yCoord = yCoord
        self.cost = 99999999999.9

    def setCost(self, newCost):
        self.cost = newCost

    def connect(self, nodeArray):
        self.connectedNodes = nodeArray

    def updateCost(self, prevNode):
        if self.cost == 0:
            return
        edge = edgeCost(self, prevNode) + prevNode.cost
        
        if(edge < self.cost):
            self.cost = edge
            self.prevNode = prevNode
    
    def setStart(self):
        self.cost = 0
        for nextNode in self.connectedNodes:
            nextNode.updateCost(self)
    
    def explore(self):
        for nextNode in self.connectedNodes:
            nextNode.updateCost(self)


nodes = []
nodes.append(Node(0,0))
nodes.append(Node(3,4))
nodes.append(Node(2,1))
nodes.append(Node(7,8))

nodes[0].connect([nodes[1], nodes[2]])
nodes[1].connect([nodes[0], nodes[2], nodes[3]])
nodes[2].connect([nodes[0], nodes[1], nodes[3]])
nodes[3].connect([nodes[1], nodes[2]])

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
        print(openList[0][1].cost)

print(openList[0][1].prevNode.cost)
print(openList[0][1].prevNode.prevNode.cost)

