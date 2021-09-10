#9/9/21
import math
import heapq # This is for the min heap library]]\

num = 1

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

    def __init__ (self, xCoord, yCoord): # so you can pss values to a new node
        self.xCoord = xCoord
        self.yCoord = yCoord
        self.cost = 99999999999.9
        self.nodeID = num
        num += 1

    def setCost(self, newCost):
        self.cost = newCost

    def connect(self, nodeArray):
        self.connectedNodes = nodeArray

    def updateCost(self, nextNode):
        edge = edgeCost(self, nextNode)

        if(edge<nextNode.cost):
            nextNode.cost = edge


node1 = Node(0,0)
node1.setCost(0)
node2 = Node(3,4)
node3 = Node(2,1)
node4 = Node(7,8)

node1.connect([node2, node3])
node2.connect([node1, node3, node4])
node3.connect([node1, node2, node4])
node4.connect([node2, node3])


openList = []
heappush(openList, )
