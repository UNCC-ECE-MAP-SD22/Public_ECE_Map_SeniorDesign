from matplotlib import pyplot as plt

label1 = {1:'A', 2:'B' , 3:'C', 4:'D', 5:'E', 6:'F'}

label2 = {'A':(-1,0), 'B': (1,2), 'C':(3,4), 'D':(2,5), 'E':(3,6), 'F':(4,4)}

connectionMatrix = [
    [1,1,1,0,1,0],
    [0,0,0,1,1,0],
    [0,0,1,1,1,0],
    [1,0,1,1,1,0],
    [0,1,1,0,0,0],
    [0,0,1,0,0,0],
]

xCoord = [label2[k][0] for k in sorted(label2)]
yCoord = [label2[k][1] for k in sorted(label2)]
for i in range(6):
    plt.plot(xCoord[i], yCoord[i], marker = "o")
    plt.text(xCoord[i]-.5, yCoord[i], label1[i+1])

for i in range(6):
    for j in range (6):
        if connectionMatrix[i][j] != 0 :
            plt.plot([xCoord[i], xCoord[j]],[yCoord[i], yCoord[j]], 'r')

plt.show()